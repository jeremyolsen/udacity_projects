/*
 * http://github.com/dusty-nv/jetson-reinforcement
*/

#include "ArmPlugin.h"
#include "PropPlugin.h"

#include "cudaMappedMemory.h"
#include "cudaPlanar.h"

#define PI 3.141592653589793238462643383279502884197169f

// Joint ranges for the arm segments and rotating base
#define JOINT_MIN      -0.75f
#define JOINT_MAX	    2.00f
#define BASE_JOINT_MIN -0.75f
#define BASE_JOINT_MAX  0.75f

// Turn on velocity based control
#define VELOCITY_CONTROL false
#define VELOCITY_MIN -0.2f
#define VELOCITY_MAX  0.2f

// Define DQN API Settings
#define INPUT_WIDTH   64
#define INPUT_HEIGHT  64
#define INPUT_CHANNELS 3
#define OPTIMIZER "RMSprop"
#define LEARNING_RATE 0.1f
#define REPLAY_MEMORY 10000
#define BATCH_SIZE 64
#define GAMMA 0.9f
#define EPS_START 0.9f
#define EPS_END 0.04f
#define EPS_DECAY 210
#define USE_LSTM true
#define LSTM_SIZE 256
#define ALLOW_RANDOM true
#define DEBUG_DQN false

// Define Object Names
#define WORLD_NAME "arm_world"
#define PROP_NAME  "tube"
#define GRIP_NAME  "gripperbase"

// Define Reward Parameters
#define REWARD_WIN   100.0f
#define REWARD_LOSS -100.0f
#define REWARD_MULTIPLIER 10.0f
#define GAMMA_FALLOFF 0.35f

// Define Collision Parameters
#define COLLISION_FILTER "ground_plane::link::collision"
#define COLLISION_ITEM   "tube::tube_link::tube_collision"
//#define COLLISION_POINT  "arm::gripper_middle::middle_collision"
#define COLLISION_POINT "arm::gripperbase::gripper_link"

// Animation Steps
#define ANIMATION_STEPS 1000

// Set Debug Mode
#define DEBUG false

// Lock base rotation DOF (Add dof in header file if off)
#define LOCKBASE true


namespace gazebo
{

    // register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ArmPlugin);


    // constructor
    ArmPlugin::ArmPlugin() : ModelPlugin(), cameraNode(new gazebo::transport::Node()), collisionNode(new gazebo::transport::Node())
    {
        printf("ArmPlugin::ArmPlugin()\n");

        agent 	       = NULL;
        inputState       = NULL;
        inputBuffer[0]   = NULL;
        inputBuffer[1]   = NULL;
        inputBufferSize  = 0;
        inputRawWidth    = 0;
        inputRawHeight   = 0;
        actionJointDelta = 0.15f;
        actionVelDelta   = 0.1f;
        maxEpisodeLength = 100;
        episodeFrames    = 0;

        newState         = false;
        newReward        = false;
        endEpisode       = false;
        rewardHistory    = 0.0f;
        testAnimation    = true;
        loopAnimation    = false;
        animationStep    = 0;
        lastGoalDistance = 0.0f;
        avgGoalDelta     = 0.0f;
        successfulGrabs  = 0;
        totalRuns        = 0;
        runHistoryIdx    = 0;
        runHistoryMax    = 0;

        // zero the run history buffer
        memset(runHistory, 0, sizeof(runHistory));

        // set the default reset position for each joint
        for( uint32_t n=0; n < DOF; n++ )
            resetPos[n] = 0.0f;

        resetPos[1] = 0.25;	 // make the arm canted forward a little

        // set the initial positions and velocities to the reset
        for( uint32_t n=0; n < DOF; n++ )
        {
            ref[n] = resetPos[n]; //JOINT_MIN;
            vel[n] = 0.0f;
        }

        // set the joint ranges
        for( uint32_t n=0; n < DOF; n++ )
        {
            jointRange[n][0] = JOINT_MIN;
            jointRange[n][1] = JOINT_MAX;
        }

        // if the base is freely rotating, set it's range separately
        if( !LOCKBASE )
        {
            jointRange[0][0] = BASE_JOINT_MIN;
            jointRange[0][1] = BASE_JOINT_MAX;
        }
    }


    // Load
    void ArmPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        printf("ArmPlugin::Load('%s')\n", _parent->GetName().c_str());

        // Create DQN agent
        if( !createAgent() )
            return;

        // Store the pointer to the model
        this->model = _parent;
        this->j2_controller = new physics::JointController(model);

        // Create our node for camera communication
        cameraNode->Init();
        cameraSub = cameraNode->Subscribe("/gazebo/" WORLD_NAME "/camera/link/camera/image", &ArmPlugin::onCameraMsg, this);

        // Create our node for collision detection
        collisionNode->Init();
        collisionSub = collisionNode->Subscribe("/gazebo/" WORLD_NAME "/" PROP_NAME "/tube_link/my_contact", &ArmPlugin::onCollisionMsg, this);

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ArmPlugin::OnUpdate, this, _1));
    }


    // CreateAgent
    bool ArmPlugin::createAgent()
    {
        if( agent != NULL )
            return true;

        // Create DQN agent
        agent = dqnAgent::Create(INPUT_WIDTH, INPUT_HEIGHT, INPUT_CHANNELS, DOF*2,
                                 OPTIMIZER, LEARNING_RATE, REPLAY_MEMORY, BATCH_SIZE,
                                 GAMMA, EPS_START, EPS_END, EPS_DECAY,
                                 USE_LSTM, LSTM_SIZE, ALLOW_RANDOM, DEBUG_DQN);

        if( !agent )
        {
            printf("ArmPlugin - failed to create DQN agent\n");
            return false;
        }

        // Allocate the python tensor for passing the camera state
        inputState = Tensor::Alloc(INPUT_WIDTH, INPUT_HEIGHT, INPUT_CHANNELS);

        if( !inputState )
        {
            printf("ArmPlugin - failed to allocate %ux%ux%u Tensor\n", INPUT_WIDTH, INPUT_HEIGHT, INPUT_CHANNELS);
            return false;
        }

        return true;
    }


    // onCameraMsg
    void ArmPlugin::onCameraMsg(ConstImageStampedPtr &_msg)
    {
        // don't process the image if the agent hasn't been created yet
        if( !agent )
            return;

        // check the validity of the message contents
        if( !_msg )
        {
            printf("ArmPlugin - recieved NULL message\n");
            return;
        }

        // retrieve image dimensions
        const int width  = _msg->image().width();
        const int height = _msg->image().height();
        const int bpp    = (_msg->image().step() / _msg->image().width()) * 8;	// bits per pixel
        const int size   = _msg->image().data().size();

        if( bpp != 24 )
        {
            printf("ArmPlugin - expected 24BPP uchar3 image from camera, got %i\n", bpp);
            return;
        }

        // allocate temp image if necessary
        if( !inputBuffer[0] || size != inputBufferSize )
        {
            if( !cudaAllocMapped(&inputBuffer[0], &inputBuffer[1], size) )
            {
                printf("ArmPlugin - cudaAllocMapped() failed to allocate %i bytes\n", size);
                return;
            }

            printf("ArmPlugin - allocated camera img buffer %ix%i  %i bpp  %i bytes\n", width, height, bpp, size);

            inputBufferSize = size;
            inputRawWidth   = width;
            inputRawHeight  = height;
        }

        memcpy(inputBuffer[0], _msg->image().data().c_str(), inputBufferSize);
        newState = true;

        if(DEBUG){printf("camera %i x %i  %i bpp  %i bytes\n", width, height, bpp, size);}

    }


    // onCollisionMsg
    void ArmPlugin::onCollisionMsg(ConstContactsPtr &contacts)
    {
        //if(DEBUG){printf("collision callback (%u contacts)\n", contacts->contact_size());}

        if( testAnimation )
            return;

        for (unsigned int i = 0; i < contacts->contact_size(); ++i)
        {
            if( strcmp(contacts->contact(i).collision2().c_str(), COLLISION_FILTER) == 0 )
                continue;

            if(DEBUG){std::cout << "Collision between[" << contacts->contact(i).collision1()
                                << "] and [" << contacts->contact(i).collision2() << "]\n";}

            std::cout << "Collision between[" << contacts->contact(i).collision1()
                      << "] and [" << contacts->contact(i).collision2() << "] - i:" << i << "\n";

            // Check if there is collision between middle prong and object then issue learning reward
            if ((strcmp(contacts->contact(i).collision1().c_str(), COLLISION_ITEM) == 0) && strcmp(contacts->contact(i).collision2().c_str(), COLLISION_POINT) == 0 && !testAnimation)
            //if ((strcmp(contacts->contact(i).collision2().c_str(), COLLISION_POINT) == 0))
            //if((strcmp(contacts->contact(i).collision1().c_str(), COLLISION_ITEM) == 0))
            {
                //if(DEBUG)
                printf("Give max reward and execute gripper \n");

                //rewardHistory = (1.0f - (float(episodeFrames) / float(maxEpisodeLength))) * REWARD_WIN;
                rewardHistory = REWARD_WIN * REWARD_MULTIPLIER * REWARD_MULTIPLIER * REWARD_MULTIPLIER;

                // Set gripper
                //j2_controller->SetJointPosition(this->model->GetJoint("gripper_right"),  0.5);
                //j2_controller->SetJointPosition(this->model->GetJoint("gripper_left"),  -0.5);

                //sleep(10);

                newReward = true;
                endEpisode = true;

                return;    // multiple collisions in the for loop above could mess with win count
            }
            else {
                // Give penalty for non correct collisions
    			//rewardHistory =  REWARD_LOSS * 0.1;
                rewardHistory = REWARD_LOSS;
                newReward  = true;
                endEpisode = true;
            }
        }
    }


    // upon recieving a new frame, update the AI agent
    bool ArmPlugin::updateAgent()
    {
        // convert uchar3 input from camera to planar BGR
        if( CUDA_FAILED(cudaPackedToPlanarBGR((uchar3*)inputBuffer[1], inputRawWidth, inputRawHeight,
                                              inputState->gpuPtr, INPUT_WIDTH, INPUT_HEIGHT)) )
        {
            printf("ArmPlugin - failed to convert %zux%zu image to %ux%u planar BGR image\n",
                   inputRawWidth, inputRawHeight, INPUT_WIDTH, INPUT_HEIGHT);

            return false;
        }

        // select the next action
        int action = 0;

        if( !agent->NextAction(inputState, &action) )
        {
            printf("ArmPlugin - failed to generate agent's next action\n");
            return false;
        }

        // make sure the selected action is in-bounds
        if( action < 0 || action >= DOF * 2 )
        {
            printf("ArmPlugin - agent selected invalid action, %i\n", action);
            return false;
        }

        if(DEBUG){printf("ArmPlugin - agent selected action %i\n", action);}

        // action 0 does nothing, the others index a joint
        /*if( action == 0 )
            return false;	// not an error, but didn't cause an update

        action--;*/	// with action 0 = no-op, index 1 should map to joint 0


        #if VELOCITY_CONTROL
        // if the action is even, increase the joint velocity by the delta parameter
        // if the action is odd,  decrease the joint velocity by the delta parameter
        float velocity = vel[action/2] + actionVelDelta * ((action % 2 == 0) ? 1.0f : -1.0f);

        if( velocity < VELOCITY_MIN )
            velocity = VELOCITY_MIN;

        if( velocity > VELOCITY_MAX )
            velocity = VELOCITY_MAX;

        vel[action/2] = velocity;

        for( uint32_t n=0; n < DOF; n++ )
        {
            ref[n] += vel[n];

            if( ref[n] < jointRange[n][0] )
            {
                ref[n] = jointRange[n][0];
                vel[n] = 0.0f;
            }
            else if( ref[n] > jointRange[n][1] )
            {
                ref[n] = jointRange[n][1];
                vel[n] = 0.0f;
            }
        }
        #else
        // index the joint, considering each DoF has 2 actions (+ and -)
        const int jointIdx = action / 2;

        // compute the new joint value and either increase or decrease it based on the action
        float joint = ref[jointIdx] + actionJointDelta * ((action % 2 == 0) ? 1.0f : -1.0f);

        // limit the joint to the specified range
        if( joint < jointRange[jointIdx][0] )
            joint = jointRange[jointIdx][0];

        if( joint > jointRange[jointIdx][1] )
            joint = jointRange[jointIdx][1];

        ref[jointIdx] = joint;
        #endif

        return true;
    }


    // update joint reference positions, returns true if positions have been modified
    bool ArmPlugin::updateJoints()
    {
        if( testAnimation )	// test sequence
        {
            const float step = (JOINT_MAX - JOINT_MIN) * (float(1.0f) / float(ANIMATION_STEPS));

            // return to base position
            for( uint32_t n=0; n < DOF; n++ )
            {
                if( ref[n] < resetPos[n] )
                    ref[n] += step;
                else if( ref[n] > resetPos[n] )
                    ref[n] -= step;

                if( ref[n] < jointRange[n][0] )
                    ref[n] = jointRange[n][0];
                else if( ref[n] > jointRange[n][1] )
                    ref[n] = jointRange[n][1];
            }

            animationStep++;

            // reset and loop the animation
            if( animationStep > ANIMATION_STEPS )
            {
                animationStep = 0;

                if( !loopAnimation )
                    testAnimation = false;
            }
            else if( animationStep == ANIMATION_STEPS / 2 )
            {
                //printf("Reset gripper \n");
                //j2_controller->SetJointPosition(this->model->GetJoint("gripper_right"),  0);
                //j2_controller->SetJointPosition(this->model->GetJoint("gripper_left"),  0);
                //RandomizeProps();
                ResetPropDynamics();
            }

            return true;
        }

        else if( newState && agent != NULL )
        {
            // update the AI agent when new camera frame is ready
            episodeFrames++;

            if(DEBUG){printf("episode frame = %i\n", episodeFrames);}

            // reset camera ready flag
            newState = false;

            if( updateAgent() )
                return true;
        }

        return false;
    }


    // get the servo center for a particular degree of freedom
    float ArmPlugin::resetPosition( uint32_t dof )
    {
        return resetPos[dof];
    }


    // compute the distance between two bounding boxes
    static float BoxDistance(const math::Box& a, const math::Box& b)
    {
	float sqrDist = 0;

	if( b.max.x < a.min.x )
	{
		float d = b.max.x - a.min.x;
		sqrDist += d * d;
	}
	else if( b.min.x > a.max.x )
	{
		float d = b.min.x - a.max.x;
		sqrDist += d * d;
	}

	if( b.max.y < a.min.y )
	{
		float d = b.max.y - a.min.y;
		sqrDist += d * d;
	}
	else if( b.min.y > a.max.y )
	{
		float d = b.min.y - a.max.y;
		sqrDist += d * d;
	}

	if( b.max.z < a.min.z )
	{
		float d = b.max.z - a.min.z;
		sqrDist += d * d;
	}
	else if( b.min.z > a.max.z )
	{
		float d = b.min.z - a.max.z;
		sqrDist += d * d;
	}

	return sqrtf(sqrDist);
}


    // called by the world update start event
    void ArmPlugin::OnUpdate(const common::UpdateInfo& updateInfo)
    {
        // deferred loading of the agent (this is to prevent Gazebo black/frozen display)
        if( !agent && updateInfo.simTime.Float() > 1.5f )
        {
            if( !createAgent() )
                return;
        }

        // verify that the agent is loaded
        if( !agent )
            return;

        // determine if we have new camera state and need to update the agent
        const bool hadNewState = newState && !testAnimation;

        // update the robot positions with vision/DQN
        if( updateJoints() )
        {
            //printf("%f  %f  %f  %s\n", ref[0], ref[1], ref[2], testAnimation ? "(testAnimation)" : "(agent)");

            double angle(1);
            //std::string j2name("joint1");

            #if LOCKBASE
            j2_controller->SetJointPosition(this->model->GetJoint("base"), 	0);
		    j2_controller->SetJointPosition(this->model->GetJoint("joint1"),  ref[0]);
		    j2_controller->SetJointPosition(this->model->GetJoint("joint2"),  ref[1]);
            #else
            j2_controller->SetJointPosition(this->model->GetJoint("base"), 	 ref[0]);
            j2_controller->SetJointPosition(this->model->GetJoint("joint1"),  ref[1]);
            j2_controller->SetJointPosition(this->model->GetJoint("joint2"),  ref[2]);
            #endif
        }

        // episode timeout
        if( maxEpisodeLength > 0 && episodeFrames > maxEpisodeLength )
        {
            printf("ArmPlugin - triggering EOE, episode has exceeded %i frames\n", maxEpisodeLength);
            rewardHistory = REWARD_LOSS;
            newReward     = true;
            endEpisode    = true;
        }

        // if an EOE reward hasn't already been issued, compute an intermediary reward
        if( hadNewState && !newReward )
        {
            // retrieve the goal prop model object
            PropPlugin* prop = GetPropByName(PROP_NAME);

            if( !prop )
            {
                printf("ArmPlugin - failed to find Prop '%s'\n", PROP_NAME);
                return;
            }

            // remember where the user moved the prop to for when it's reset
            //prop->UpdateResetPose();

            // get the bounding box for the prop object
            const math::Box& propBBox = prop->model->GetBoundingBox();
            physics::LinkPtr gripper  = model->GetLink(GRIP_NAME);

            if( !gripper )
            {
                printf("ArmPlugin - failed to find Gripper '%s'\n", GRIP_NAME);
                return;
            }

            // if the robot impacts the ground, count it as a loss
            const math::Box& gripBBox = gripper->GetBoundingBox();
            const float groundContact = 0.01f;

            if( gripBBox.min.z <= groundContact || gripBBox.max.z <= groundContact )
            {
                //for( uint32_t n=0; n < 10; n++ )
                printf("GROUND CONTACT, EOE\n");

                rewardHistory = REWARD_LOSS;
                newReward     = true;
                endEpisode    = true;
            }
            else
            {
                const float distGoal = BoxDistance(gripBBox, propBBox); // compute the reward from distance to the goal

                if(DEBUG){printf("distance('%s', '%s') = %f\n", gripper->GetName().c_str(), prop->model->GetName().c_str(), distGoal);}

                // issue an interim reward based on the delta of the distance to the object
                if( episodeFrames > 1 )
                {
                    const float distDelta  = lastGoalDistance - distGoal;
                    const float distThresh = 0.50f;		// maximum distance to the goal
                    const float epsilon    = 0.001f;		// minimum pos/neg change in position
                    const float movingAvg  = 0.01f;//0.9f;

                    // compute the smoothed moving average of the delta of the distance to the goal
                    avgGoalDelta  = (avgGoalDelta * movingAvg) + (distDelta * (1.0f - movingAvg));
                    rewardHistory = avgGoalDelta * (exp(-GAMMA_FALLOFF * distGoal) * 0.1f) * REWARD_MULTIPLIER;
                    newReward     = true;
                }

                lastGoalDistance = distGoal;
            }
        }

        // issue rewards and train DQN
        if( newReward && agent != NULL )
        {
            printf("ArmPlugin - issuing reward %f, EOE=%s  %s\n", rewardHistory, endEpisode ? "true" : "false", (rewardHistory > 0.1f) ? "POS+" :(rewardHistory > 0.0f) ? "POS" : (rewardHistory < 0.0f) ? "    NEG" : "       ZERO");
            agent->NextReward(rewardHistory, endEpisode);

            // reset reward indicator
            newReward = false;

            // reset for next episode
            if( endEpisode )
            {
                testAnimation    = true;	// reset the robot to base position
                loopAnimation    = false;
                endEpisode       = false;
                episodeFrames    = 0;
                lastGoalDistance = 0.0f;
                avgGoalDelta     = 0.0f;

                // track the number of wins and agent accuracy
                if( rewardHistory >= REWARD_WIN )
                {
                    runHistory[runHistoryIdx] = true;
                    successfulGrabs++;
                }
                else
                    runHistory[runHistoryIdx] = false;

                const uint32_t RUN_HISTORY = sizeof(runHistory);
                runHistoryIdx = (runHistoryIdx + 1) % RUN_HISTORY;
			    totalRuns++;

                printf("%s  wins = %03u of %03u (%0.2f)  ", (rewardHistory >= REWARD_WIN) ? "WIN " : "LOSS", successfulGrabs, totalRuns, float(successfulGrabs)/float(totalRuns));

                if( totalRuns >= RUN_HISTORY )
                {
                    uint32_t historyWins = 0;

                    for( uint32_t n=0; n < RUN_HISTORY; n++ )
                    {
                        if( runHistory[n] )
                            historyWins++;
                    }

                    if( historyWins > runHistoryMax )
                        runHistoryMax = historyWins;

                    printf("%02u of last %u  (%0.2f)  (max=%0.2f)", historyWins, RUN_HISTORY, float(historyWins)/float(RUN_HISTORY), float(runHistoryMax)/float(RUN_HISTORY));
                }

                printf("\n");

			    printf("Current Accuracy:  %0.4f (%03u of %03u)  (reward=%+0.2f %s)\n", float(successfulGrabs)/float(totalRuns), successfulGrabs, totalRuns, rewardHistory, (rewardHistory >= REWARD_WIN ? "WIN" : "LOSS"));

                //printf("Reset gripper \n");
                //j2_controller->SetJointPosition(this->model->GetJoint("gripper_right"),  0);
                //j2_controller->SetJointPosition(this->model->GetJoint("gripper_left"),  0);
                //ResetPropDynamics();  // now handled mid-reset sequence

                for( uint32_t n=0; n < DOF; n++ )
                    vel[n] = 0.0f;
            }
        }
    }

}
