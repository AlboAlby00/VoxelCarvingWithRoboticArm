// /*** UMAIR: **/
#include "common.h"
#include <random>

void colourVoxels(std::vector<camera> &cameras, std::vector<voxel> &voxels)
{
    for (auto &voxel : voxels)
    {
        float runningSumR = 0.0;
        float runningSumG = 0.0;
        float runningSumB = 0.0;
        int numContributingCameras = 0;

        for (auto &camera : cameras)
        {
            cv::Mat voxel_coord(4, 1, CV_32F);
            voxel_coord.at<float>(0) = voxel.xpos;
            voxel_coord.at<float>(1) = voxel.ypos;
            voxel_coord.at<float>(2) = voxel.zpos;
            voxel_coord.at<float>(3) = 1.0f;

            cv::Mat pixel_coord;
            cv::gemm(camera.P, voxel_coord, 1.0, cv::Mat(), 0.0, pixel_coord);
            // cv::Mat pixel_coord = cam.P * voxel_coord;

            coord im;
            im.x = pixel_coord.at<float>(0) / pixel_coord.at<float>(2);
            im.y = pixel_coord.at<float>(1) / pixel_coord.at<float>(2);

            if (im.x >= 0 && im.x < IMG_WIDTH &&
                im.y >= 0 && im.y < IMG_HEIGHT &&
                camera.Silhouette.at<uchar>(im.y, im.x) != 0 && voxel.value <= 0)
            {
                cv::Vec3b pixelColor = camera.Image.at<cv::Vec3b>(im.y, im.x);
                runningSumB += pixelColor[0];
                runningSumG += pixelColor[1];
                runningSumR += pixelColor[2];
                numContributingCameras++;
            }
        }

        if (numContributingCameras > 0)
        {
            voxel.red = runningSumR / numContributingCameras;
            voxel.green = runningSumG / numContributingCameras;
            voxel.blue = runningSumB / numContributingCameras;
            std::cout << "Voxel (" << voxel.xpos << ", " << voxel.ypos << ", " << voxel.zpos << ") has colour " << voxel.red << " " << voxel.green << " " << voxel.blue << "\n";
        }
    }
}

// /*********/

// void colourVoxels(unsigned char *colourData)
// {
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<int> dis(0, 255);

//     for (int i = 0; i < VOXEL_DIM; i++)
//     {
//         for (int j = 0; j < VOXEL_DIM; j++)
//         {
//             for (int k = 0; k < VOXEL_DIM; k++)
//             {
//                 int voxelIndex = i * VOXEL_DIM * VOXEL_DIM + j * VOXEL_DIM + k;
//                 int colorIndex = voxelIndex * 3;

//                 // Set the color values for the current voxel
//                 // Generate random color values for the current voxel
//                 unsigned char red = static_cast<unsigned char>(dis(gen));
//                 unsigned char green = static_cast<unsigned char>(dis(gen));
//                 unsigned char blue = static_cast<unsigned char>(dis(gen));

//                 // Assign the color values to the colorData array
//                 colourData[colorIndex] = red;
//                 colourData[colorIndex + 1] = green;
//                 colourData[colorIndex + 2] = blue;
//             }
//         }
//     }
// }
