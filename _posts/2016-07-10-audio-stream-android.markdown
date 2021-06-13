---
layout: archive
title:  "Audio streaming in android. Part 1"
date:   2016-07-10 13:42:41 +0100
visible: false
---

I want to build an android app to stream songs from one device to another, with a server in between. To do this, the method I'm going to use is when you pick a song to stream, the app divides the song in frames and send them to the server with a socket, the the server send the data to each client that want to listen to the song, and in the app I decode the frames into PCM and use AudioTrack to play the audio. I'm going to use AudioTrack instead of the usual MediaPlayer because of the latency to change from one fragment of audio to another.


<h1>Player</h1>

I'm going to start with the player, where I'm going to start with a .wav file, which I will read with a sleep(time) to simulate any network and process time that it will have in the future. 

<h1>First try</h1>

{% highlight java %}
public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        /* ... */

        AudioDevice device = new AudioDevice( );
        int bufferSize = 1<<20;
        byte bigbuffer[] = new byte[bufferSize];
        int k = 0;

        ByteArrayOutputStream ous = null;
        FileInputStream ios = null;
        try {
            byte[] buffer = new byte[16384];
            ous = new ByteArrayOutputStream();
            ios = new FileInputStream("/sdcard/song.wav");
            int read = 0;
            while ((read = ios.read(buffer)) != -1) {

                if (buffer.length + k > bufferSize) {
                    device.writeSamples( bigbuffer );
                    k = 0;
                    Thread.sleep(2000);
                }

                for( int i = 0; i < buffer.length; i++ )
                    bigbuffer[k++] = (buffer[i]);;

                //Thread.sleep(100);

            }

        }catch(FileNotFoundException err) {
            err.printStackTrace();
        } catch(IOException err) {
            err.printStackTrace();
        } catch(InterruptedException err) {
            err.printStackTrace();
        } finally {
            try {
                if (ous != null)
                    ous.close();
            } catch (IOException e) {
            }

            try {
                if (ios != null)
                    ios.close();
            } catch (IOException e) {
            }
        }


    }

    public class AudioDevice
    {
        AudioTrack track;
        byte[] buffer = new byte[4096];

        public AudioDevice( )
        {
            int minSize =AudioTrack.getMinBufferSize( 44100, AudioFormat.CHANNEL_CONFIGURATION_STEREO, AudioFormat.ENCODING_PCM_16BIT );
            track = new AudioTrack( AudioManager.STREAM_MUSIC, 44100,
                    AudioFormat.CHANNEL_CONFIGURATION_STEREO, AudioFormat.ENCODING_PCM_16BIT,
                    minSize, AudioTrack.MODE_STREAM);
            track.play();
        }

        public void writeSamples(byte[] samples)
        {
            fillBuffer( samples );
            track.write( buffer, 0, samples.length );
        }

        private void fillBuffer( byte[] samples )
        {
            if( buffer.length < samples.length )
                buffer = new byte[samples.length];

            for( int i = 0; i < samples.length; i++ )
                buffer[i] = (samples[i]);;
        }
}

{% endhighlight %}

This is the basic algorithm, I read the file and store the data in a buffer of around 1MB, which then I will send to the AudioTrack, which will then play it. This solution has a big problem, and is that AudioTrack function write is blocking, so it will not return until it has stopped playing all the data that has been written. In order to solve this, the AudioTrack write should be in a separate thread than the file write. We also would have needed to do this when we implement the read from the server. To do this I'm going to use the fact that if we also put the file reading in a thread, one thread will only write and the other one will only read, so I'm going to create a kind of shared queue, where the writing thread will push the data at the end, and the read thread will pop the data from the beginning.

<h1>Shared Queue</h1>

{% highlight java %}
class SharedQueue
{
    public static SharedQueue globalInstance = new SharedQueue();

    private byte[] buffer;
    private int start;
    private int index;
    private int size;
    private int used;
    private volatile boolean isReading;
    private volatile boolean isResizing;

    public SharedQueue() {
        buffer = new byte[1<<20];
        start = 0;
        size = 1<<20;
        index = 0;
        used = 0;
    }

    public void write( byte[] data ) throws InterruptedException
    {
        if( size-used < data.length )
        {
            while (isReading) {
                Thread.sleep(50);
            }

            isResizing = true;
            int localStart = start;
            int localUsed = used;
            isResizing = false;

            int newSize = Math.max(2*size, size + data.length);
            byte[] newBuffer = new byte[newSize];
            
            for (int i=0; i<localUsed; i++) {
                newBuffer[i] = buffer[(localStart+i)%size];
            }
            

            while (isReading) {
                Thread.sleep(50);
            }

            isResizing = true;
            buffer = newBuffer;
            if (start >= localStart) {
                start = start - localStart;
            }
            else {
                start = size - localStart + start;
            }
            size = newSize;
            index = start + used;
            isResizing = false;
        }

        for( int i = 0; i < data.length; i++ )
        {
            buffer[index] = (data[i]);
            index = (index + 1)%size;
        }
        add(data.length);
    }

    public int size() {
        return used;
    }

    public void add(int val) {
        synchronized(this) {
            used += val;
        }
    }

    public boolean read(byte[] data, int len) throws InterruptedException
    {
        if (size() < len) return false;

        while (isResizing) {
            Thread.sleep(10);
        }

        isReading = true;
        add(-len);
        for (int i=0; i<len; i++) {
            data[i] = buffer[(start+i)%size];
        }
        start = (start+len)%size;
        isReading = false;

        return true;
    }

}
{% endhighlight %}

In the code above, I have used a dynamic array to store the queue, where it will act in a circular fashion, if I have reached the end of the array, but I have space at the beginning, I will use this space to store data, and when I used all the space, I resize the array and copy all the data to the new array. Obviously, there are some part of the functions that can't be done in parallel with other functions, like the resizing, that can't happen at the same time of a read. Right now, I can't think of an easy way to do this, so I wait until a flag variable is unset, and marks that a conflicting operations has ended. Luckily, most of the work can be done in parallel, for example, in the resize, all the data copy that consumes most of the time, doesn't affect anything else, only the reset of the variables. 
Using this data structure, we finally obtain a working solution that will play the song without any lag and pauses inside the song.

{% highlight java %}

        /* ... */

        new Thread( new Runnable( )
        {
            public void run( )
            {
                AudioDevice device = new AudioDevice( );
                int bufferSize = 1<<20;
                byte bigbuffer[] = new byte[bufferSize];
                int k = 0;

                ByteArrayOutputStream ous = null;
                FileInputStream ios = null;
                try {
                    byte[] buffer = new byte[16384];
                    ous = new ByteArrayOutputStream();
                    ios = new FileInputStream("/sdcard/song.wav");
                    int read = 0;
                    while ((read = ios.read(buffer)) != -1) {

                        if (buffer.length + k > bufferSize) {

                            SharedQueue.globalInstance.write( bigbuffer );
                            k = 0;
                            Thread.sleep(2000);
                        }

                        for( int i = 0; i < buffer.length; i++ )
                            bigbuffer[k++] = (buffer[i]);;

                    }

                }catch(FileNotFoundException err) {
                    err.printStackTrace();
                } catch(IOException err) {
                    err.printStackTrace();
                } catch(InterruptedException err) {
                    err.printStackTrace();
                } finally {
                    try {
                        if (ous != null)
                            ous.close();
                    } catch (IOException e) {
                    }

                    try {
                        if (ios != null)
                            ios.close();
                    } catch (IOException e) {
                    }
                }
            }
        } ).start();

    }

    public class AudioDevice
    {
        AudioTrack track;
        byte[] buffer = new byte[4096];

        public AudioDevice( )
        {
            int minSize =AudioTrack.getMinBufferSize( 44100, AudioFormat.CHANNEL_CONFIGURATION_STEREO, AudioFormat.ENCODING_PCM_16BIT );
            track = new AudioTrack( AudioManager.STREAM_MUSIC, 44100,
                    AudioFormat.CHANNEL_CONFIGURATION_STEREO, AudioFormat.ENCODING_PCM_16BIT,
                    minSize, AudioTrack.MODE_STREAM);
            track.play();

            new Thread( new Runnable( ) {
                public void run() {

                    while (true) {
                        if (SharedQueue.globalInstance.size() >= buffer.length) {
                            try {
                                SharedQueue.globalInstance.read(buffer, buffer.length);
                                track.write(buffer, 0, buffer.length);
                            }catch (InterruptedException e) {

                            }
                        }
                    }
                }
            } ).start();
        }

        /* ... */

{% endhighlight %}

While doing this I realized than in Java array copying is very slow, so instead of this, I used NDK to write the copy functions in C++ improving the performance a lot.

{% highlight cpp %}

JNIEXPORT void JNICALL
Java_com_example_alex_audiostream_JNIWrapper_copybyteArrayJni(JNIEnv *env, jobject obj, jbyteArray arr, jbyteArray buf)
{
    jsize len = (*env)->GetArrayLength(env, arr);
    int i, sum = 0;
    jint *a = (*env)->GetIntArrayElements(env, arr, 0);
    jint *b = (*env)->GetIntArrayElements(env, buf, 0);
    for (i=0; i<len; i++) {
        b[i] = a[i];
    }
    (*env)->ReleaseIntArrayElements(env, arr, a, 0);
    (*env)->ReleaseIntArrayElements(env, buf, b, 0);
}

JNIEXPORT void JNICALL
Java_com_example_alex_audiostream_JNIWrapper_copyByteArrayModularJni(JNIEnv *env, jobject obj, jbyteArray from, jbyteArray to, jint start, jint len)
{
    jsize size = (*env)->GetArrayLength(env, from);
    int i, sum = 0;
    jbyte *a = (*env)->GetByteArrayElements(env, from, 0);
    jbyte *b = (*env)->GetByteArrayElements(env, to, 0);
    for (i=0; i<len; i++) {
        b[i] = a[(i+start)%size];
    }
    (*env)->ReleaseByteArrayElements(env, from, a, 0);
    (*env)->ReleaseByteArrayElements(env, to, b, 0);
}

{% endhighlight %}

This functions substitute the array copying implemented in the read and resize in the SharedQueue.



With this I have a solution that I think will be able to play the streamed audio.