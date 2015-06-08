
#include "ImageRegistration.h"

ImageRegistration::ImageRegistration() :
    sizeColor(1920, 1080), sizeIr(512, 424), depthShift(0), maxDepth(12.0),
    depthRegHighRes(DepthRegistration::New(DepthRegistration::OPENCL))
{
    ;
}

ImageRegistration::~ImageRegistration()
{
    delete depthRegHighRes;
}

void ImageRegistration::init(const std::string &calib_path, const std::string &sensor)
{
    initCalibration(calib_path, sensor);
}

void ImageRegistration::initCalibration(const std::string &calib_path, const std::string &sensor)
{
    std::string calibPath = calib_path + sensor + '/';
    std::cout << "folder: " << calibPath.c_str() << std::endl;
    struct stat fileStat;
    bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
    std::cout << "calibDirNotFound " << calibDirNotFound << std::endl;
    if (calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor)) {
        std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
    }

    if (calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr)) {
        std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
    }

    if (calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation)) {
        std::cerr << "using defaults for rotation and translation." << std::endl;
    }

    if (calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift)) {
        std::cerr << "using defaults for depth shift." << std::endl;
        depthShift = 0.0;
    }

    depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, -1);
    
    const int mapType = CV_16SC2;
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);

    std::cout << std::endl << "camera parameters used:" << std::endl
              << "camera matrix color:" << std::endl << cameraMatrixColor << std::endl
              << "distortion coefficients color:" << std::endl << distortionColor << std::endl
              << "camera matrix ir:" << std::endl << cameraMatrixIr << std::endl
              << "distortion coefficients ir:" << std::endl << distortionIr << std::endl
              << "rotation:" << std::endl << rotation << std::endl
              << "translation:" << std::endl << translation << std::endl
              << "depth shift:" << std::endl << depthShift << std::endl
              << std::endl;
}

bool ImageRegistration::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ)) {
        fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
        fs[K2_CALIB_DISTORTION] >> distortion;
        fs.release();
    } else {
        std::cerr << "can't open calibration file: " << filename << std::endl;
        return false;
    }
    return true;
}

bool ImageRegistration::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ)) {
        fs[K2_CALIB_ROTATION] >> rotation;
        fs[K2_CALIB_TRANSLATION] >> translation;
        fs.release();
    } else {
        std::cerr << "can't open calibration pose file: " << filename << std::endl;
        return false;
    }
    return true;
}

bool ImageRegistration::loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
{
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ)) {
        fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
        fs.release();
    } else {
        std::cerr << "can't open calibration depth file: " << filename << std::endl;
        return false;
    }
    return true;
}

void ImageRegistration::register_images(const cv::Mat &color, const cv::Mat &ir_depth, cv::Mat &out) const
{
    cv::Mat color_flipped;
    cv::flip(color, color_flipped, 1);
    cv::Mat color_rect;
    cv::remap(color_flipped, color_rect, map1Color, map2Color, cv::INTER_AREA);

    cv::Mat ir_depth_shifted;

    ir_depth.convertTo(ir_depth_shifted, CV_16U, 1, depthShift);

    cv::flip(ir_depth_shifted, ir_depth_shifted, 1);

    //cv::Mat depth_shifted_rect;
    //cv::remap(ir_depth_shifted, depth_shifted_rect, map1Ir, map2Ir, cv::INTER_NEAREST);

    depthRegHighRes->registerDepth(ir_depth_shifted, out);
}