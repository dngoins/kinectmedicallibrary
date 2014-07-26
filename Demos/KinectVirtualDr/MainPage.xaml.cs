using KincectDoctor.Common;
using KincectDoctor.Data;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Windows.Input;
using Windows.ApplicationModel.Resources;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using WindowsPreview.Kinect;
using KinectMedicalLibrary;
using System.Numerics;
using AForge.Math;
using Bing.Speech;
using Windows.UI.Core;


namespace KincectDoctor
{

    public sealed partial class MainPage : Page
    {
        private readonly uint bytesPerPixel;
        private KinectSensor sensor = null;
        private CoordinateMapper coordinateMapper = null;
        private MultiSourceFrameReader multiFrameReader = null;
        private WriteableBitmap wBitmap = null;
        private Stream stream = null;
        private byte[] colorPixels = null;
        private short[] depthPixels = null;
        private ushort[] irData = null;
        private int[] matrixCounter = new int[7];
        private IRMatrix[,] matrices;
        bool firstActivation = true;
        bool currentlyTakingVitals = false;
        bool currentlyShowingResults = false;

        /// <summary>
        /// Stream for 32b-16b conversion.
        /// </summary>
        private KinectAudioStream convertStream = null;

        /// <summary>
        /// Speech recognition engine using audio data from Kinect.
        /// </summary>
        private SpeechRecognizer speechEngine = null;

        private float[,] irAverages ;

        private HeartRate m_heartRate = null;
        private bool needsMoreFrames = true;
        private int HeartRateCount = 0;

        private Body[] bodies = null;

        TextBlock[] StatusTexts = new TextBlock[6];
        private const int cDepthWidth = 512;
        private const int cDepthHeight = 424;
        private const int cInfraredWidth = 512;
        private const int cInfraredHeight = 424;
        private const int cColorWidth = 1920;
        private const int cColorHeight = 1080;
        private const int cFramesPerSecond = 30;
        private const int cHeartRateTime = 60;
        private const int cMintesToRecord = 2;
        private const int maxSeconds = 1800;
        Windows.Storage.Streams.IInputStream  audioStream;
        private int showStartScreenCounter = 0;

        public MultiSourceFrame[] hrData = new MultiSourceFrame[60];
        int j = 0;
        public MainPage()
        {

            this.sensor = KinectSensor.GetDefault();
            this.multiFrameReader = this.sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.BodyIndex);
            this.multiFrameReader.MultiSourceFrameArrived += this.AllFrameAvailable;

            FrameDescription colorFrameDescription = this.sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);
            this.bytesPerPixel = colorFrameDescription.BytesPerPixel;
            this.colorPixels = new byte[colorFrameDescription.Width * colorFrameDescription.Height * this.bytesPerPixel];
            this.wBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height);
            this.stream = this.wBitmap.PixelBuffer.AsStream();

            this.bodies = new Body[this.sensor.BodyFrameSource.BodyCount];
            this.coordinateMapper = this.sensor.CoordinateMapper;
            FrameDescription frameDescription = this.sensor.DepthFrameSource.FrameDescription;

            FrameDescription irFrameDescription = this.sensor.InfraredFrameSource.FrameDescription;
            this.irData = new ushort[irFrameDescription.Width * irFrameDescription.Height];

            this.sensor.Open();


            this.DataContext = this;
            this.InitializeComponent();

            m_heartRate = new HeartRate();
            this.matrices = new IRMatrix[maxSeconds, 7];
            this.irAverages = new float[maxSeconds, 7];
            //this. .Children.Add(this.drawingCanvas);

            // grab the audio stream
            IReadOnlyList<AudioBeam> audioBeamList = this.sensor.AudioSource.AudioBeams;
            this.audioStream = audioBeamList[0].OpenInputStream();

            // create the convert stream
            this.convertStream = new KinectAudioStream(audioStream);

            Window.Current.Activated += Window_Activated;
        }

        void speechEngine_RecognizerResultReceived(SpeechRecognizer sender, SpeechRecognitionResultReceivedEventArgs args)
        {
            throw new NotImplementedException();
        }

        private void AllFrameAvailable(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            bool processed = true;
            bool dataReceived = true;
            using (MultiSourceFrame multiFrame = e.FrameReference.AcquireFrame())
            {
                #region Image Display
                using (ColorFrame colorFrame = multiFrame.ColorFrameReference.AcquireFrame())
                {
                    if (multiFrame != null && colorFrame != null)
                    {
                        hrData[j] = multiFrame;
                        j++;
                        if (j == 60)
                        {


                            j = 0;
                        }

                        FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.wBitmap.PixelWidth) && (colorFrameDescription.Height == this.wBitmap.PixelHeight))
                        {
                            if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                            {
                                colorFrame.CopyRawFrameDataToArray(this.colorPixels);
                            }
                            else
                            {
                                colorFrame.CopyConvertedFrameDataToArray(this.colorPixels, ColorImageFormat.Bgra);
                            }

                            processed = true;
                        }
                    }

                    // we got a frame, render
                    if (processed)
                    {
                        RenderColorPixels(this.colorPixels);
                    }

                #endregion
                    #region Body Tracking
                    using (BodyFrame bodyFrame = multiFrame.BodyFrameReference.AcquireFrame())
                    {
                        if (bodyFrame != null)
                        {
                            bodyFrame.GetAndRefreshBodyData(this.bodies);

                            dataReceived = true;
                        }
                    }
                    if (dataReceived)
                    {
                        StatusTexts[0] = PersonStatus1;
                        StatusTexts[1] = PersonStatus2;
                        StatusTexts[2] = PersonStatus3;
                        StatusTexts[3] = PersonStatus4;
                        StatusTexts[4] = PersonStatus5;
                        StatusTexts[5] = PersonStatus6;

                        for (int bodyIndex = 0; bodyIndex < this.bodies.Length - 1; bodyIndex++)
                        {
                            Body body = this.bodies[bodyIndex];
                            if (body != null && body.IsTracked)
                            {
                                
                              //  mainImage.Visibility = Visibility.Visible;
                                StatusTexts[bodyIndex].Visibility = Visibility.Visible;

                                if (!this.currentlyTakingVitals)
                                {
                                    
                                    VisualStateManager.GoToState(this, VSG_TakingVitals.Name, false);
                                    //mainImage.Opacity = 1;
                                    //this.storyboard.Stop();
                                    //this.takeVitals.Begin();
                                    this.currentlyTakingVitals = true;
                                }
                                
                                //Get Facial expressions
                                var expressions = body.Expressions;
                                //foreach(var expressionItem in expressions)
                                //{
                                //    var expression = expressionItem.Value;
                                //    var expName = expressionItem.Key;

                                    
                                //}

                                //var appearanceItem = body.Appearance;

                                //foreach (var appearanceItem in body.Appearance)
                                //{
                                //    var appearance = appearanceItem.Value;
                                //    var appName = appearanceItem.Key;
                                //}

                                //var activities = body.Activities;
                                //foreach(var activityItem in activities)
                                //{
                                //    var activity = activityItem.Value;
                                //    var actName = activityItem.Key;
                                  
                                //}

                                var jointPointsInDepthSpace = new Dictionary<JointType, Point>();
                                Joint head = body.Joints[JointType.Head];

                                Rect headRct, chestRct;
                                var foundHead = GetHeadRegion(body, ref headRct);

                                if (foundHead)
                                {
                                    //start calculating heart Rate
                                    using (InfraredFrame irFrame = multiFrame.InfraredFrameReference.AcquireFrame())
                                    {
                                        if (irFrame != null)
                                        {
                                            var timestamp = irFrame.RelativeTime;
                                           
                                            irFrame.CopyFrameDataToArray(this.irData);
                                            var avg = GetAverageIRIntensity(headRct, this.irData);
                                            this.irAverages[matrixCounter[bodyIndex], bodyIndex] = avg;
                                            this.matrices[matrixCounter[bodyIndex], bodyIndex].BodyIndex = bodyIndex;
                                            this.matrices[matrixCounter[bodyIndex], bodyIndex].AverageIRIntensity = avg;
                                            this.matrices[matrixCounter[bodyIndex], bodyIndex].TimeStamp = timestamp;

                                            if (matrixCounter[bodyIndex] == 0)
                                            {
                                                this.matrices[matrixCounter[bodyIndex], bodyIndex].AverageIRIntensity = 0;
                                                this.matrices[matrixCounter[bodyIndex], bodyIndex].RateOfChange = 0;

                                            }
                                            else
                                            {
                                                var timeDelta = this.matrices[matrixCounter[bodyIndex], bodyIndex].TimeStamp - this.matrices[matrixCounter[bodyIndex] - 1, bodyIndex].TimeStamp;
                                                var freqDelta = this.matrices[matrixCounter[bodyIndex], bodyIndex].AverageIRIntensity - this.matrices[matrixCounter[bodyIndex] - 1, bodyIndex].AverageIRIntensity;
                                                var rateOfChange = freqDelta / timeDelta.Milliseconds;
                                                this.matrices[matrixCounter[bodyIndex], bodyIndex].RateOfChange = rateOfChange;


                                            }

                                            matrixCounter[bodyIndex]++;

                                            //every 6 seconds, or 1800 frames calculate a value...
                                            if ((matrixCounter[bodyIndex]) % 256 == 0)
                                            {
                                                AForge.Math.Complex[] cmpxArray = new AForge.Math.Complex[256];

                                                var cnt = matrixCounter[bodyIndex] - 256;
                                                for (int i = 0; i < 256; i++)
                                                {
                                                    cmpxArray[i] = new AForge.Math.Complex(this.irAverages[cnt, bodyIndex], 0);

                                                    cnt++;


                                                }

                                                var cmplx = Hann(cmpxArray);
                                                FourierTransform.FFT(cmplx, FourierTransform.Direction.Forward);
                                                double bpm = 0;
                                                for (int j = 1; j < cmpxArray.Length; j++)
                                                {
                                                    if (cmpxArray[j].Re > cmpxArray[j - 1].Re)
                                                    {
                                                        bpm = cmpxArray[j].Re * 9;
                                                        //was 12;
                                                    }
                                                }
                                                HeartRateCount = (int)bpm;

                                                StatusTexts[bodyIndex].Text = string.Format("{0} bpm", HeartRateCount);

                                                if(!currentlyShowingResults && currentlyTakingVitals)
                                                {


                                                    VisualStateManager.GoToState(this, VSG_ShowResults.Name, true);
                                                        //this.takeVitals.Stop();
                                                        this.currentlyTakingVitals = false;
                                                        this.title5.Text = string.Format("Person tracked at: {0}", bodyIndex);
                                                        this.title6.Text = string.Format("has a HeartRate Of {0} bpm", HeartRateCount);
                                                        //this.showVitalResults.Begin();
                                                        currentlyShowingResults = true;

                                                   
                                                }
                                            }

                                            if (matrixCounter[bodyIndex] >= maxSeconds)
                                            {
                                                //    //we're finished counting...

                                                VisualStateManager.GoToState(this, VSG_ShowResults.Name, true);
                                                    
                                                matrixCounter[bodyIndex] = 0;
                                            }

                                            //    var size = matrixCounter[bodyIndex] - 1;
                                            //    var calcAvg = AverageRateOfChange(size, bodyIndex);

                                            //    var calcStdDev = Deviation(calcAvg, size, bodyIndex);
                                            //    var heartRateCount = CalculateRateSpikesMoreThan3Deviations(calcAvg, calcStdDev, size, bodyIndex);
                                            //    HeartRateCount = heartRateCount;
                                            //    StatusTexts[bodyIndex].Text = string.Format("{0} bpm", HeartRateCount);
                                            //    matrixCounter[bodyIndex] = 0;
                                            //}


                                        }
                                    }

                                }
                                // var foundChest = GetChestRegion(body, ref chestRct);

                                CameraSpacePoint position = body.Joints[JointType.Head].Position;

                                DepthSpacePoint depthSpacePoint = coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPointsInDepthSpace[JointType.Head] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                                
                                
                                StatusTexts[bodyIndex].SetValue(Canvas.TopProperty, 140 + jointPointsInDepthSpace[JointType.Head].Y);
                                StatusTexts[bodyIndex].SetValue(Canvas.LeftProperty, 180 + jointPointsInDepthSpace[JointType.Head].X);
                                showStartScreenCounter = 0;
                                VisualStateManager.GoToState(this, VSG_TakingVitals.Name , true);
                                           
                            }
                            else
                            {
                                //if
                                //if(currentlyShowingResults || currentlyTakingVitals)
                                //{
                                  //  VisualStateManager.GoToState(this, VSG_Attract.Name, true);
                                    //this.showVitalResults.Stop();
                                    //this.takeVitals.Stop();
                                    //this.storyboard.Begin();
                                    //this.currentlyShowingResults = false;
                                    //this.currentlyTakingVitals = false;
                                   // 
                               // }
                                showStartScreenCounter++;
                                if(showStartScreenCounter > 600)
                                {
                                    VisualStateManager.GoToState(this, VSG_Attract.Name , true);
                                    showStartScreenCounter = 0;
                                }
                                StatusTexts[bodyIndex].Visibility = Visibility.Collapsed;
                                // mainImage.Visibility = Visibility.Collapsed;
                            }
                        }
                    }
                    else
                        VisualStateManager.GoToState(this, VSG_Attract.Name, true);
                }
                #endregion

                //#region "Heart Rate Section"

                //using (BodyFrame bodyFrame = multiFrame.BodyFrameReference.AcquireFrame())
                //{
                //    if (bodyFrame == null)
                //    {
                //        return;                        
                //    }
                //    using (BodyIndexFrame bodyIndexFrame = multiFrame.BodyIndexFrameReference.AcquireFrame())
                //    {
                //        if (bodyIndexFrame == null)
                //        {
                //            return;
                //        }


                //        using (InfraredFrame irFrame = multiFrame.InfraredFrameReference.AcquireFrame())
                //        {
                //            if (irFrame == null)
                //            {
                //                return;
                //            }

                //            needsMoreFrames = m_heartRate.GetCurrentHeartRate(bodyFrame, bodyIndexFrame, irFrame);
                //        }
                //    }
                //}
                //#endregion

            }
       }
        private void RenderColorPixels(byte[] pixels)
        {
            stream.Seek(0, SeekOrigin.Begin);
            stream.Write(pixels, 0, pixels.Length);
            wBitmap.Invalidate();
            mainImage.Source = wBitmap;



        }

        private Point BodyToScreen(CameraSpacePoint bodyPoint, int width, int height)
        {
            // Calculate the body's position on the screen
            DepthSpacePoint depthPoint = this.coordinateMapper.MapCameraPointToDepthSpace(bodyPoint);
            float screenPointX = (depthPoint.X * width) / cDepthWidth;
            float screenPointY = (depthPoint.Y * height) / cDepthHeight;

            return new Point(screenPointX, screenPointY);
        }

        private bool GetHeadRegion(Body body, ref Rect temprct)
        {
            foreach (var jointItem in body.Joints)
            {
                var joint = jointItem.Value;
                var jointType = jointItem.Key;
                var pt = BodyToScreen(joint.Position, (int)this.canvasImage.Width, (int)this.canvasImage.Height);

                if (jointType == JointType.Head)
                {

                    var x = (double)(pt.X - 50);
                    var width = (double)(pt.X + 50);
                    var y = (double)(pt.Y - 50);
                    var height = (double)(pt.Y + 50);
                    temprct = new Rect(x, y, width, height);
                    return true;
                }

            }

            return false;

        }

        private bool GetChestRegion(Body body, ref Rect temprct)
        {
            foreach (var jointItem in body.Joints)
            {
                var joint = jointItem.Value;
                var jointType = jointItem.Key;
                var pt = BodyToScreen(joint.Position, (int)this.canvasImage.Width, (int)this.canvasImage.Height);

                if (jointType == JointType.SpineMid)
                {

                    var x = (double)(pt.X - 100);
                    var width = (double)(pt.X + 100);
                    var y = (double)(pt.Y - 100);
                    var height = (double)(pt.Y + 100);
                    temprct = new Rect(x, y, width, height);
                    return true;
                }


            }



            return false;

        }

        private float GetAverageIRIntensity(Rect rct, ushort[] pBuffer)
        {
            //const UINT16* pBufferEnd = pBuffer + (cInfraredHeight * cInfraredWidth);
            int maxIRSize = (int)(rct.Bottom - rct.Top) * (int)(rct.Right - rct.Left) + 1;
            if (maxIRSize > this.irData.Length) maxIRSize = this.irData.Length;

            float intensity = 0.0f;
            float sumIntensity = 0.0f;
            long sumPixelsCounted = 0;

            for (int i = 0; i < maxIRSize; i++)
            {

                if ((i >= (rct.Top + 1)) && (i <= (rct.Bottom * rct.Right)))
                {
                    //Only do this for Players tracked within Head Region

                    int ir = pBuffer[i];

                    sumIntensity = sumIntensity + ir;
                    ++sumPixelsCounted;
                }

            }
            intensity = sumIntensity / sumPixelsCounted;
            return intensity;
        }

        private float AverageRateOfChange(int size, int bodyIndex)
	{
			float sum = 0;

            for (int i = 0; i < size; i++)
            {
                if(this.matrices[i, bodyIndex].BodyIndex == bodyIndex)
                    sum += Math.Abs(this.matrices[i, bodyIndex].RateOfChange);
            }
			return sum / (float)size;
	}

        float Deviation( float ave, int size, int bodyIndex)
	{ 
		float E = 0;
		//ASSERT(v.size());
		float inverse = (float)(1.0 / (size -1.0 ));
		for (int i = 0; i<size ; i++)
		{
            if (this.matrices[i, bodyIndex].BodyIndex == bodyIndex)
			E += (float)Math.Pow((this.matrices[i, bodyIndex].RateOfChange  - ave), 2.0);
		}
		return (float)Math.Sqrt(inverse * E);
	}

        int CalculateRateSpikesMoreThan3Deviations(float average, float deviation,  int size, int bodyIndex)
	{
		var count = 0;
		for ( int i = 0; i < size; i++)
		{
            if (this.matrices[i, bodyIndex].BodyIndex == bodyIndex)
            {
                float deviationRatio = Math.Abs((this.matrices[i, bodyIndex].RateOfChange - average) / deviation);
                if (deviationRatio > 2.0)
                    count++;
            }
		}
		return count * 2;
	}

        public AForge.Math.Complex[] Hann (AForge.Math.Complex[] iwv)
        {
            int N = iwv.Length;
            for (int n = 1; n < N; n++)
                iwv[n].Re = 0.5f * (float)Math.Cos((2 * Math.PI * n) / (N - 1));
            return iwv;
        }

        void Window_Activated(object sender, WindowActivatedEventArgs e)
        {
            // Code that only runs for the first activation:
            if (this.firstActivation)
            {
                StartInitialAnimations();
                this.firstActivation = false;
            }
        }

        void StartInitialAnimations()
        {
            // Start the animation
            VisualStateManager.GoToState(this, VSG_Attract.Name, true);
        }
    }
}
     