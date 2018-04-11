/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Sammy Pfeiffer
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/sync_queue.hpp>

#include <mutex>











// --------------------------------------------------------------------
// Awesome code by BloodAxe (https://github.com/BloodAxe) - found here:
// --------------------------------------------------------------------
// https://computer-vision-talks.com/hacking-opencv-for-fun-and-profit/
// --------------------------------------------------------------------

// modules\core\include\opencv2\core.hpp
struct CV_EXPORTS MemorySnapshot
{
    //! Total amount of allocated memory.
    size_t allocatedMemory;

    //! Maximum amount of allocated memory for the whole time.
    size_t peakMemoryUsage;

    //! Maximum amount of allocated memory since last snapshot.
    size_t peakMemoryUsageSinceLastSnapshot;
    
    //! Number of memory allocations count for the whole program running time.
    size_t allocationsCount;

    //! Number of memory deallocations for the whole program running time.
    size_t deallocationsCount;
    
    //! Number of allocated objects that are still live (e.g not deallocated).
    size_t liveObjects;
};

CV_EXPORTS MemorySnapshot memorySnapshot();

// modules\core\alloc.cpp
class MemoryManager
{
public:
    //! Key - pointer to allocated memory, Value - it's size
    typedef std::map<void*, size_t>     AllocationTable;
    typedef std::lock_guard<std::mutex> LockType;

    void recordAlloc(void* ptr, size_t size)
    {
        LockType guard(mAllocMutex);
        mAllocatedMemory.insert(std::make_pair(ptr, size));

        mCurrentMemoryUsage += size;
        mPeakMemoryUsage = std::max(mPeakMemoryUsage, mCurrentMemoryUsage);
        mPeakMemoryUsageSinceLastSnapshot = std::max(mPeakMemoryUsageSinceLastSnapshot, mCurrentMemoryUsage);
        mAllocationsCount++;
    }

    void recordFree(void* ptr)
    {
        LockType guard(mAllocMutex);

        auto block = mAllocatedMemory.find(ptr);
        CV_Assert(block != mAllocatedMemory.end());
    
        mCurrentMemoryUsage -= block->second;
        mDeallocationsCount++;
        mAllocatedMemory.erase(block);
    }

   

    static MemoryManager& Instance()
    {
        std::call_once(mInitFlag, []() {
            if (mInstance == nullptr)
            {
                mInstance = new MemoryManager();
            }
        });

        return *mInstance;
    }

    MemorySnapshot makeSnapshot()
    {
        LockType guard(mAllocMutex);
        
        MemorySnapshot snapshot;
        snapshot.peakMemoryUsage = mPeakMemoryUsage;
        snapshot.peakMemoryUsageSinceLastSnapshot = mPeakMemoryUsageSinceLastSnapshot;
        snapshot.allocatedMemory = mCurrentMemoryUsage;
        snapshot.allocationsCount = mAllocationsCount;
        snapshot.deallocationsCount = mDeallocationsCount;
        snapshot.liveObjects = mAllocationsCount - mDeallocationsCount;
        
        mPeakMemoryUsageSinceLastSnapshot = 0;

        return std::move(snapshot);
    }
private:

    MemoryManager()
        : mCurrentMemoryUsage(0)
        , mPeakMemoryUsage(0)
        , mPeakMemoryUsageSinceLastSnapshot(0)
        , mAllocationsCount(0)
        , mDeallocationsCount(0)
    {
    }

private:
    std::mutex      mAllocMutex;
    AllocationTable mAllocatedMemory;

    size_t          mCurrentMemoryUsage;
    size_t          mPeakMemoryUsage;
    size_t          mPeakMemoryUsageSinceLastSnapshot;

    size_t          mAllocationsCount;
    size_t          mDeallocationsCount;

    static std::once_flag  mInitFlag;
    static MemoryManager * mInstance;
};

std::once_flag  MemoryManager::mInitFlag;
MemoryManager * MemoryManager::mInstance = nullptr;


#define CV_MALLOC_ALIGN 64

template<typename T> inline T* alignPtr(T* ptr, size_t n=sizeof(T))
{
    return (T*)(((size_t)ptr + n-1) & -n);
}


static void* OutOfMemoryError(size_t size)
{
    CV_Error_(CV_StsNoMem, ("Failed to allocate %llu bytes", (unsigned long long)size));
    return 0;
}

namespace cv {
		// modules\core\alloc.cpp
		void* fastMalloc(size_t size)
		{
		    uchar* udata = (uchar*)malloc(size + sizeof(void*) + CV_MALLOC_ALIGN);
		
		    if(!udata)
		        return OutOfMemoryError(size);
		
		    MemoryManager::Instance().recordAlloc(udata, size);
		    uchar** adata = alignPtr((uchar**)udata + 1, CV_MALLOC_ALIGN);
		    adata[-1] = udata;
		    return adata;
		}
		
		void fastFree(void* ptr)
		{
		    if(ptr)
		    {
		        uchar* udata = ((uchar**)ptr)[-1];
		        CV_DbgAssert(udata < (uchar*)ptr &&
		               ((uchar*)ptr - udata) <= (ptrdiff_t)(sizeof(void*)+CV_MALLOC_ALIGN));
		
		        MemoryManager::Instance().recordFree(udata);
		        free(udata);
		    }
		}
}

MemorySnapshot memorySnapshot()
{
    return std::move(MemoryManager::Instance().makeSnapshot());
}

#define MEASURE_MEMORY(x) { size_t memOnStart = memorySnapshot().allocatedMemory;			\
                            x;										\
                            size_t memOnEnd   = memorySnapshot().allocatedMemory;			\
                            std::cout << #x << "\t" << memOnStart << "/" << memOnEnd << std::endl; }


// --------------------------------------------------------------------
// End of awesome code by BloodAxe. Ok I just modified it a bit just to
// avoid to recompile the whole OpenCV, but it seems to work ok anyway.
// --------------------------------------------------------------------
// https://computer-vision-talks.com/hacking-opencv-for-fun-and-profit/
// --------------------------------------------------------------------








boost::sync_queue<cv::Mat> framesQueue;
cv::VideoCapture cap;
std::string camera_name;
double set_camera_fps;
int max_queue_size;

// Based on the ros tutorial on transforming opencv images to Image messages

sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr img){
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;
    ROS_INFO_STREAM("The image width is: " << img->width);
    ROS_INFO_STREAM("The image height is: " << img->height);
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(1.0) (0.0) (img->width/2.0)
                                           (0.0) (1.0) (img->height/2.0)
                                           (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (img->width/2.0) (0.0)
                                            (0.0) (1.0) (img->height/2.0) (0.0)
                                            (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
}


void do_capture(ros::NodeHandle &nh) {
    cv::Mat frame;
    cv::Mat _drop_frame;

    ros::Rate camera_fps_rate(set_camera_fps);

    // Read frames as fast as possible
    while (nh.ok()) {
        cap >> frame;
	if (camera_name == "videofile")
	{
         camera_fps_rate.sleep();
	}
        if(!frame.empty()) {
            // accumulate only until max_queue_size
            if (framesQueue.size() < max_queue_size) {
                MEASURE_MEMORY(framesQueue.push(frame.clone()));
            }
            // once reached, drop the oldest frame
            else {
		std::cout << "Queue before framesQueue.pull(_drop_frame): " << framesQueue.size() << std::endl;
                MEASURE_MEMORY(framesQueue.pull(_drop_frame));
		std::cout << "Queue after framesQueue.pull(_drop_frame): " << framesQueue.size() << std::endl;
                MEASURE_MEMORY(framesQueue.push(frame.clone()));
		std::cout << "Queue after framesQueue.push(frame.clone()): " << framesQueue.size() << std::endl;
            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~"); // to get the private params
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("camera", 1);

    // provider can be an url (e.g.: rtsp://10.0.0.1:554) or a number of device, (e.g.: 0 would be /dev/video0)
    std::string video_stream_provider;
    if (_nh.getParam("video_stream_provider", video_stream_provider)){
        ROS_INFO_STREAM("Resource video_stream_provider: " << video_stream_provider);
        // If we are given a string of 4 chars or less (I don't think we'll have more than 100 video devices connected)
        // treat is as a number and act accordingly so we open up the videoNUMBER device
        if (video_stream_provider.size() < 4){
            ROS_INFO_STREAM("Getting video from provider: /dev/video" << video_stream_provider);
            cap.open(atoi(video_stream_provider.c_str()));
        }
        else{
            ROS_INFO_STREAM("Getting video from provider: " << video_stream_provider);
            cap.open(video_stream_provider);
        }
    }
    else{
        ROS_ERROR("Failed to get param 'video_stream_provider'");
        return -1;
    }

    _nh.param("camera_name", camera_name, std::string("camera"));
    ROS_INFO_STREAM("Camera name: " << camera_name);
    
    _nh.param("set_camera_fps", set_camera_fps, 30.0);
    ROS_INFO_STREAM("Setting camera FPS to: " << set_camera_fps);
    cap.set(CV_CAP_PROP_FPS, set_camera_fps);
    
    double reported_camera_fps;
    // OpenCV 2.4 returns -1 (instead of a 0 as the spec says) and prompts an error
    // HIGHGUI ERROR: V4L2: Unable to get property <unknown property string>(5) - Invalid argument
    reported_camera_fps = cap.get(CV_CAP_PROP_FPS);
    if (reported_camera_fps > 0.0)
        ROS_INFO_STREAM("Camera reports FPS: " << reported_camera_fps);
    else
        ROS_INFO_STREAM("Backend can't provide camera FPS information");
    
    int buffer_queue_size;
    _nh.param("buffer_queue_size", buffer_queue_size, 100);
    ROS_INFO_STREAM("Setting buffer size for capturing frames to: " << buffer_queue_size);
    max_queue_size = buffer_queue_size;

    double fps;
    _nh.param("fps", fps, 240.0);
    ROS_INFO_STREAM("Throttling to fps: " << fps);
    
    if (video_stream_provider.size() < 4 && fps > set_camera_fps)
        ROS_WARN_STREAM("Asked to publish at 'fps' (" << fps
        << ") which is higher than the 'set_camera_fps' (" << set_camera_fps <<
        "), we can't publish faster than the camera provides images.");

    std::string frame_id;
    _nh.param("frame_id", frame_id, std::string("camera"));
    ROS_INFO_STREAM("Publishing with frame_id: " << frame_id);

    std::string camera_info_url;
    _nh.param("camera_info_url", camera_info_url, std::string(""));
    ROS_INFO_STREAM("Provided camera_info_url: '" << camera_info_url << "'");

    bool flip_horizontal;
    _nh.param("flip_horizontal", flip_horizontal, false);
    ROS_INFO_STREAM("Flip horizontal image is: " << ((flip_horizontal)?"true":"false"));

    bool flip_vertical;
    _nh.param("flip_vertical", flip_vertical, false);
    ROS_INFO_STREAM("Flip vertical image is: " << ((flip_vertical)?"true":"false"));

    int width_target;
    int height_target;
    _nh.param("width", width_target, 0);
    _nh.param("height", height_target, 0);
    if (width_target != 0 && height_target != 0){
        ROS_INFO_STREAM("Forced image width is: " << width_target);
        ROS_INFO_STREAM("Forced image height is: " << height_target);
    }

    // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
    // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
    bool flip_image = true;
    int flip_value;
    if (flip_horizontal && flip_vertical)
        flip_value = -1; // flip both, horizontal and vertical
    else if (flip_horizontal)
        flip_value = 1;
    else if (flip_vertical)
        flip_value = 0;
    else
        flip_image = false;

    if(!cap.isOpened()){
        ROS_ERROR_STREAM("Could not open the stream.");
        return -1;
    }
    if (width_target != 0 && height_target != 0){
        cap.set(CV_CAP_PROP_FRAME_WIDTH, width_target);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, height_target);
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;
    header.frame_id = frame_id;
    camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
    // Get the saved camera info if any
    cam_info_msg = cam_info_manager.getCameraInfo();
    
    ROS_INFO_STREAM("Opened the stream, starting to publish.");
    bool first_run = true;
    boost::thread cap_thread(do_capture, nh);

    ros::Rate r(fps);
    while (nh.ok()) {
	if (!framesQueue.empty())
	{
		std::cout << "Queue before framesQueue.pull(frame): " << framesQueue.size() << std::endl;
                MEASURE_MEMORY(framesQueue.pull(frame));
		std::cout << "Queue after framesQueue.pull(frame): " << framesQueue.size() << std::endl;
	}

        if (pub.getNumSubscribers() > 0){
            // Check if grabbed frame is actually filled with some content
            if(!frame.empty()) {
                // Flip the image if necessary
                if (flip_image)
                    cv::flip(frame, frame, flip_value);
                msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                // Create a default camera info if we didn't get a stored one on initialization
                if (cam_info_msg.distortion_model == ""){
                    ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
                    cam_info_msg = get_default_camera_info_from_image(msg);
                    cam_info_manager.setCameraInfo(cam_info_msg);
                }
		if (first_run) {
                   first_run = false;
                   uchar depth = frame.type() & CV_MAT_DEPTH_MASK;
                   uchar chans = 1 + (frame.type() >> CV_CN_SHIFT);
			std::cout << uint32_t(frame.type()) << std::endl;
			std::cout << uint32_t(depth) << std::endl;
			std::cout << uint32_t(chans) << std::endl;
			std::cout << uint32_t(sizeof(frame.data)) << std::endl;
			std::cout << uint32_t(frame.data) << std::endl;
			std::cout << uint32_t(frame.datastart) << std::endl;
			std::cout << uint32_t(frame.dataend) << std::endl;
			std::cout << uint32_t(frame.datalimit) << std::endl;

                   uint64_t memusage = 1.0 * max_queue_size * (uint32_t(frame.dataend) - uint32_t(frame.datastart)) / 1048576;

                   if (memusage > 512) // MB of RAM
                   {
                      ROS_WARN_STREAM("WARNING. Queue size: " << max_queue_size << " * allocated image size: "
			<< uint32_t(frame.dataend) - uint32_t(frame.datastart)
			<< " may occupy up to: " << memusage << " MB of RAM");
                      ROS_WARN_STREAM("especially on slow CPUs like on Raspberry Pi where exact timings cannot be met.");
                      ROS_WARN_STREAM("Reduce the queue size if memory consumption becomes too high.");
                   }
		}
                // The timestamps are in sync thanks to this publisher
                pub.publish(*msg, cam_info_msg, ros::Time::now());
            }

            ros::spinOnce();
        }
        r.sleep();
    }
    cap_thread.join();
}
