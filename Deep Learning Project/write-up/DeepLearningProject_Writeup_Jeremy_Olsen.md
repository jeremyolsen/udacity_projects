#Udacity - Robotics Software Engineer Nanodegree - Deep Learning Project

**Jeremy Olsen - slack: jolsen-oas**

[//]: #parameterseferences
[image1]: ./img/hand_drawn_archtecture.jpg

Follow-Me Network Architecture
---

**Purpose**                             
The purpose of this network is to perform Semantic Segmentation on the input from an RGB camera.  Semantic Segmentation is the process of understanding and prediction of every individual pixel in an image.  We can accomplish this by creating a a form of metadata called feature maps that describe each pixel. 

**Description**

The Follow-Me project is a deep learning exercise that is using a Fully Convolutional Network (FCN) architecture.  The model design I've settled with consists of 7 network layers.  These are broken down by 3 encoding layers, the 1x1 convolution layer, and 3 decoder layers.  The network is going to be processing images with an input size of 160x160x3. Starting filter depth in the first layer was determined to be 32, as suggested by the project files.  This will result in a final filter depth of 256 that the 1x1 convolution is processing.

The project is writen using the Keras module. The module acts as a wrapper to the TensorFlow library which performs many of the management and setup routines that is required to build Tensorflow networks.  I opted to run this project on my local Nvidia GPU using the Tensorflow GPU library.  I feel that I could achieved better results if I had used the AWS cloud instead.

**Hand drawn diagram of Follow-Me Project**
![Hand drawn diagram of Follow-Me Project][image1]

The architecture of the FCN can be looked at a high level and be described by the encoder and decoder layers. Starting with the encoder layer, it is effectively breaking down the image input and outputting feature maps as an output of a 4D Tensor for each layer. This output is fed into a 1x1 convolution layer which is acting as a mini-neural network on each pixel of the image. The decoder layer performs bi-linear up-scaling on the output of each encoder layer and connects it back to the original with the final output the same size of the original image, but with much more depth.   

The FCN is a more recent addition to the field of computer vision and is well suited to Semantic Segmentation b/c of it's inherent ability to maintain spatial information.  This is in contrast to a Convolutional Neural Networks (CNN) which preceded the FCN.  CNNs when compared to FCNs, are deficient when dealing with dimensionality. The output from regular convolutions are 2D Tensors and can only be used as a simple classifier for the results.  This classification does not contain the 'where' it originated from.
  
Semantic Segmentation is an extremely interesting field as it's constantly evolving.  I'm going to include a link to a blog post about the different areas of research being conducted and was some of the basis to my conclusions.  

[A 2017 Guide to Semantic Segmentation with Deep Learning](http://blog.qure.ai/notes/semantic-segmentation-deep-learning-review)

Network Hyper Parameters
---
The following are descriptions of the hyper parameters that were configured and tuned in this project.

**'learning_rate' :** A high learning rate means that bigger steps are taken in the weight updates and thus, it may take less time for the model to converge on an optimal set of weights

**'batch_size' :** The number of images that get processed by the network in a single pass.

**'num_epochs' :** The number of times the whole dataset of images is processed though the network.

**'steps_per_epoch' :** The number of batches of images that are processed by the network in 1 epoch.  

**'validation_steps' :** The number of batches of validation images that go through the network in 1 epoch. This is similar to steps_per_epoch, except validation_steps is for the validation dataset. We have provided you with a default value for this as well.

**'workers' :** maximum number of threads / processes to spin up.  Recommended was 2.  I played with increasing this value as I'm running the GPU version of Tensorflow locally and found that it dramatically increased the speed of my training run.  I opted to leave this alone for my final training run as I was skeptical of what results it might be causing downstream.

**Network Tuning Experience**

I was able to achieve a 46.9% accuracy as my high score.  I used brute force to  obtain my results.  The correlation that I found most helpful when training the network was that the more images I added, the accuracy would start to decrease.  By adding more steps_per_epoch and corresponding number of validation steps using a 4:1 ratio, I would increase accuracy.  The num_epochs ended up staying fairly consistent no matter what the other params were.

The following are the values I settled on for the final training run that I submitting for review.

```python             
learning_rate = 0.002 
batch_size = 5        
num_epochs = 15       
steps_per_epoch = 1000
validation_steps = 250
workers = 2           
```

Network Layer Types
---
**1x1 Convolutions**

A regular or classic convolution is just a linear classifier and results in 2D ouput.  A 1x1 convolution, which is looking a single pixel with n number of filters, it is acting as a mini-neural network when added to a convolutional network.  This is also known as a reduction in dimensionality technique when placed somewhere in a network.


**Fully Connected Layers**

From the classroom: A "Fully Connected" layer is a standard, non convolutional layer, where all inputs are connected to all output neurons. This is also referred to as a "dense" layer.

Encoding & Decoding Techniques
---

A more in depth look at the encoding and decoding processes:

**Encoding**

   * Purpose: Extract features from the image
   * Uses separable convolutions vs regular convolutions
     * Regular convolutions, one filter must be applied for each input channel which leads to N filters for n channels.  The drawback is that they produce a large number number of parameters and result in high memory usage
     * Separable Convolutions reduce the number of features of each layer.  An arbitrary number of filters is applied to each input channel.  Each input channel gets traversed with 1 filter each.  The feature maps are then traversed by N number of 1x1 convolutions.
   * Batch normalization is performed after each convolution.
     * The process of normalizing the outputs after each layer vs. just normalizing the initial inputs to the network which helps train networks faster.

**Decoding**

   * Purpose: Merge features back onto the spatial data.
   * Up-scales the output of each corresponding encoding layer by a factor of 2 using bi-linear up-sampling
   * Merges it with corresponding encoded layer (the skip connection).       
   * Followed by another set of separable convolutions are performed to extract more spatial data.
     * Needed because trainable parameters are required inside the layer for the network to learn.


Limitations of the Model and Data
---

The following question was posed in the rubric: Can the model and data be used to identify and follow an animal or some other object instead of a human?  Because of the fact that identifying features will be different between the objects, the current model would work and data will not work.  We would have to retrain the model using image sets that includes animals and cars, or what ever object we are searching for.

Enhancements
---

If I had more time to apply to this project to increase accuracy, some of the things I would do differently would be:

  * Use AWS instead of using my local GPU
    * Increase the amount of memory I could consume and would help facilitate the following.
  * Increase nearly all of the hyper parameters to allow for great data coverage and faster processing which would hopefully result in better accuracy.
  * Create more encoder / decoder layers resulting in larger filter depths.
  * Potentially add more normalization in each layer.
  * Increase the size of the dataset of images substantially.  
  
 