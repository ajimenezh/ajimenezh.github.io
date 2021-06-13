---
layout: archive
title:  "Music content-based filtering recommendation"
date:   2016-03-22 13:42:41 +0100
categories: 
visible: 1
---

In a content-based recommender system, we use attributes to describe the items and use these attributes to calculate how similar are two items. With this we can rank other items on how similar they are and recommend the most similar ones. This algorithm recommend items similar to the items of the user, without using information about the action of other users.

In music recommendation, tipically Collaborative Filtering (CF) gives much better results, but there are some cases where you don't have enough information, for example in unpopular or new items, for this it's necessary to use other methods.

<h2>Item representation</h2>

The items we are working with are audio snippets (or full songs). We are going to transform these into a fixed-size vector that we can work with and use as input into a classifier. We are going to use a bag-of-words representation, in other words, we are going to build a dictionary of "patterns", and decompose the audio in these words.

The first step is to extract the <a href="https://en.wikipedia.org/wiki/Mel-frequency_cepstrum">Mel-frequency cepstral coefficients (MFCCs)</a> from the audio signal


{% highlight python %}
from pydub import AudioSegment
import numpy as np
import scipy.io.wavfile
from scikits.talkbox.features import mfcc

def convert_from_mp3_to_wav(input, output):
    song = AudioSegment.from_mp3(input)
    song.export(output, format="wav")

from os import walk

def mfcc(file, update):

  if not os.path.exists(file[0:-4] + ".npy") or update:

      convert_from_mp3_to_wav(file, "tmp.wav")

      sample_rate, X = scipy.io.wavfile.read("tmp.wav")
      ceps, mspec, spec = mfcc(X, nwin=512, nfft=1024, fs=22050, nceps=13)

      np.save(file[0:-4], ceps) 

{% endhighlight %}

In the code above, first we need to convert the audio to .wav (in this example we use .mp3) and then we can obtain the mfcc with scikits. This is very slow, so its better to cache the result.

After this, to build the bag-of-words, we are going to learn n elements with the k-means algorithm and then assign all the MFCC vector to the closest mean.

{% highlight python %}
import numpy as np
from sklearn.cluster import MiniBatchKMeans
import pickle

    def build_MFCC(self, data):

        ceps = np.load(data)
        num_ceps = len(ceps)

        X = np.array(ceps).tolist()

        h = 1.0

        # First and second order differences
        for i in range(num_ceps):

            for j in range(len(ceps[0])):

                if i == num_ceps-1:

                    X[i].append((ceps[num_ceps-1][j] - ceps[num_ceps-2][j]) / h)

                else:

                    X[i].append((ceps[i+1][j] - ceps[i][j]) / 2.0 / h)

        for i in range(num_ceps):

            for j in range(len(ceps[0])):

                if i==0:

                    X[i].append((ceps[2][j] - 2*ceps[1][j] + ceps[0][j])/h/h)

                elif i==num_ceps-1:

                    X[i].append((ceps[num_ceps-1][j] - 2*ceps[num_ceps-2][j] + ceps[num_ceps-3][j])/h/h)

                else:

                    X[i].append((ceps[i+1][j] - ceps[i][j] + ceps[i-1][j])/h/h)

        return X

if rebuild or not os.path.exists("MiniBatchKMeans"):

    mbk = MiniBatchKMeans(init='k-means++', n_clusters=4000, batch_size=1000,
              n_init=10, max_no_improvement=10, verbose=0)

    for (dirpath, dirnames, filenames) in walk(".\\MFCC"):
        for file in filenames:

            y = self.build_MFCC(".\\MFCC\\" + file)

            mbk.partial_fit( y )

        break

    s = pickle.dumps(mbk)
    
    file = open("MiniBatchKMeans", "w")
    file.write(s)
    file.close()

{% endhighlight %}

We use MiniBatchKMeans because its faster, and because its "online", the memory it uses is less than in KMeans. If we used the KMeans with a lot of data, you'll probably find memory limit errors.

Before doing the k-means, we compute also the first and second order differences of the MFCC to obtain better results.

Here we also cache the result using <a href="https://docs.python.org/2/library/pickle.html">pickle</a>.

Then, for each song, we assigne each MFCC vector to the closest mean and count how many time each mean appear in tge song. This will be the bag of words representation of the song. In order to reduce the data, we use Principal Component Analysis to reduce the size of the representation

{% highlight python %}
from sklearn.decomposition import PCA

if rebuild or not os.path.exists("PCA"):

    pca = PCA(n_components=50)
    
    file = open("MiniBatchKMeans")
    mbk = pickle.load(file)

    for (dirpath, dirnames, filenames) in walk(".\\MFCC"):
        for file in filenames:

            y = self.build_MFCC(".\\MFCC\\" + file)

            v = mbk.predict(y)

            cnt = [0]*mbk.n_clusters

            for i in v:
                cnt[i] += 1

            X.append(cnt)

        break

    pca.fit(X)

    s = pickle.dumps(pca)

    file = open("PCA", "w")
    file.write(s)
    file.close()
{% endhighlight %}

Here we build the PCA using the song representation, and then the final function would look something like this:

{% highlight python %}

def calc_song_features(self, file):

    y = self.build_MFCC(file)

    file = open("MiniBatchKMeans")
    mbk = pickle.load(file)

    file = open("PCA")
    pca = pickle.load(file)

    v = mbk.predict(y)

    cnt = [0]*mbk.n_clusters

    for i in v:
        cnt[i] += 1

    result = pca.transform(cnt)

    return result[0]
    
{% endhighlight %}

<h2>Recommender algorithm</h2>

Here we already have a vector representation of every song, with this we could calculate the similarity of two songs with the euclidean distance of the two vectors
and rank all the songs based on this distance to obtain the most similar items.

{% highlight python %}
def calc_distance(self, v, w):

    return np.linalg.norm(v-w)
{% endhighlight %}

This should be done offline, and store some number m of similar items for fast query.

But, as said before, a purely content-based recommendation does not give good result based on the user perspective, so in order to improve the results we can use a metric learning algorithm with the CF data to obtain a weighted distance function.

Briefly, CF consists of factorizing a matrix which stores if a song is liked or has been listened by a user, for ezample with <a href="https://github.com/MrChrisJohnson/implicit-mf/blob/master/mf.py">https://github.com/MrChrisJohnson/implicit-mf/blob/master/mf.py</a>. After this we can train the MFCC vectors we have obtained with a mixture of the results obtained with the euclidean distance and CF, for example, Support Vector Machines (SVM) seems to give good results. With this we can mix the good results that are obtained with CF with the possibilites to rank new data.