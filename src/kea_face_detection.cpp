#include <iostream>
#include <vector>
#include <exception>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "facedetectcnn.h"

#include <chronoptics/tof/kea_camera.hpp>
#include <chronoptics/tof/user_config.hpp>
#include <chronoptics/tof/camera_config.hpp>
#include <chronoptics/tof/gige_interface.hpp>
#include <chronoptics/tof/usb_interface.hpp>

#include "cxxopts.hpp"
#include "colormap.hpp"

namespace tof = chronoptics::tof;
#define DETECT_BUFFER_SIZE 0x20000

int get_frame(std::vector<tof::Data> &frames, const tof::FrameType frame_type)
{
    // Get the index of a frame.
    int ind = 0;
    for (tof::Data &frame : frames)
    {
        if (frame.frame_type() == frame_type)
        {
            return ind;
        }
        ind++;
    }
    return -1;
}

int main(int argc, char *argv[])
{
    cxxopts::Options options("Kea camera face detection",
                             "Using the Projected RGB data to detect the face display the face location on the depth image.");

    // Setup ToF Camera
    std::string serial = "202002a";
    float dmax;
    float fps;
    cv::String bgr_name = "RGB Face Detection";
    cv::String rad_name = "Depth Face Detection";

    options.add_options()("h, help", "Help")("l, list", "List all cameras discovered")("dmax", "Maximum distance", cxxopts::value<float>(dmax)->default_value("30.0"))("fps", "Depth frames per second", cxxopts::value<float>(fps)->default_value("15.0"))("serial", "Camera Serial Number", cxxopts::value<std::string>(serial));

    auto result = options.parse(argc, argv);

    if (result.count("h") || result.count("help"))
    {
        std::cout << options.help() << std::endl;
        return 0;
    }

    if (result.count("l") || result.count("list"))
    {
        std::vector<tof::DiscoveryMessage> cameras;
        std::vector<tof::UsbDevice> devices;

        devices = tof::usb_device_discover();
        tof::GigeInterface gige;
        cameras = gige.discover();

        for (std::size_t i = 0; i < devices.size(); i++)
        {
            std::cout << devices[i].serial() << " - usb" << std::endl;
        }

        for (std::size_t i = 0; i < cameras.size(); i++)
        {
            auto &msg = cameras[i];
            std::cout << msg.serial() << " " << msg.ip() << std::endl;
        }
        return 0;
    }
    // XXX : The Intensity did not work with this library for Face Detection.
    // std::vector<tof::FrameType> types = {tof::FrameType::RADIAL, tof::FrameType::INTENSITY};
    std::vector<tof::FrameType> types = {tof::FrameType::BGR_PROJECTED, tof::FrameType::Z};

    int *pResults = NULL;
    // pBuffer is used in the detection functions.
    // If you call functions in multiple threads, please create one buffer for each thread!
    unsigned char *pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
    if (!pBuffer)
    {
        fprintf(stderr, "Can not alloc buffer.\n");
        return -1;
    }

    try
    {
        tof::KeaCamera cam(tof::ProcessingConfig{}, serial);
        // tof::KeaCamera cam(tof::ProcessingConfig{});

        // Going to use use_config to do stuff ...
        tof::UserConfig user{};
        user.set_fps(fps);
        user.set_integration_time(tof::IntegrationTime::MEDIUM);
        user.set_max_distance(dmax);
        user.set_environment(tof::ImagingEnvironment::INSIDE);
        user.set_strategy(tof::Strategy::BALANCED);

        auto config = user.to_camera_config(cam);
        auto proc = config.default_processing();
        proc.set_intensity_scale(5.0f);

        // This should be comming soon.
        cam.set_camera_config(config);
        cam.set_process_config(proc);

        auto stream_list = cam.get_stream_list();
        // Check if BGR in the stream list.

        size_t nstreams = tof::select_streams(cam, types);
        if (nstreams != 2)
        {
            throw std::runtime_error("Require two output streams from camera, do you have an RGB camera?");
        }

        cv::namedWindow(bgr_name, cv::WINDOW_NORMAL);
        cv::namedWindow(rad_name, cv::WINDOW_NORMAL);

        std::array<std::array<uint8_t, 3>, 256> jet;
        populate_colormap(jet);

        cam.start();
        cv::Mat rad_disp;

        while (cam.is_streaming())
        {
            std::vector<tof::Data> frames = cam.get_frames();
            int z_ind = get_frame(frames, tof::FrameType::Z);

            for (auto &frame : frames)
            {
                if (frame.frame_type() == tof::FrameType::BGR_PROJECTED)
                {

                    cv::Mat rad_img(frames.at(z_ind).rows(), frames.at(z_ind).cols(), CV_32FC1, frames.at(z_ind).data());
                    convert_image_float(rad_img, rad_disp, 0.0f, dmax * 1000.0, jet);
                    cv::flip(rad_disp, rad_disp, 0);

                    cv::Mat bgr_img(frame.rows(), frame.cols(), CV_8UC3, frame.data());
                    cv::flip(bgr_img, bgr_img, 0);

                    cv::TickMeter cvtm;
                    cvtm.start();

                    // Now pass into Face detection
                    pResults = facedetect_cnn(pBuffer, (unsigned char *)(bgr_img.ptr(0)), bgr_img.cols, bgr_img.rows, (int)bgr_img.step);

                    cvtm.stop();
                    printf("time = %gms\n", cvtm.getTimeMilli());

                    for (int i = 0; i < (pResults ? *pResults : 0); i++)
                    {
                        short *p = ((short *)(pResults + 1)) + 142 * i;
                        int confidence = p[0];
                        int x = p[1];
                        int y = p[2];
                        int w = p[3];
                        int h = p[4];

                        // show the score of the face. Its range is [0-100]
                        char sScore[256];
                        snprintf(sScore, 256, "%d", confidence);
                        cv::putText(rad_disp, sScore, cv::Point(x, y - 3), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

                        // draw face rectangle
                        cv::rectangle(rad_disp, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
                        // draw five face landmarks in different colors
                        cv::circle(rad_disp, cv::Point(p[5], p[5 + 1]), 1, cv::Scalar(255, 0, 0), 2);
                        cv::circle(rad_disp, cv::Point(p[5 + 2], p[5 + 3]), 1, cv::Scalar(0, 0, 255), 2);
                        cv::circle(rad_disp, cv::Point(p[5 + 4], p[5 + 5]), 1, cv::Scalar(0, 255, 0), 2);
                        cv::circle(rad_disp, cv::Point(p[5 + 6], p[5 + 7]), 1, cv::Scalar(255, 0, 255), 2);
                        cv::circle(rad_disp, cv::Point(p[5 + 8], p[5 + 9]), 1, cv::Scalar(0, 255, 255), 2);

                        // print the result
                        printf("face %d: confidence=%d, [%d, %d, %d, %d] (%d,%d) (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",
                               i, confidence, x, y, w, h,
                               p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14]);
                    }

                    cv::imshow(bgr_name, bgr_img);
                    cv::imshow(rad_name, rad_disp);

                    if (cv::waitKey(10) == 27)
                    {
                        cam.stop();
                    }
                }
            }
        }
    }
    catch (std::exception &e)
    {
        free(pBuffer);
        cv::destroyAllWindows();
        std::cout << e.what() << std::endl;
        return -1;
    }
    cv::destroyAllWindows();

    // Close everything down
    free(pBuffer);
}
