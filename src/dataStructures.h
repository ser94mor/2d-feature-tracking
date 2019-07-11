#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

/* common macroses representing separators */
#define COMMA                ,
#define PLUS                 +
#define EMPTY

/* common macroses representing macro-functions */
#define ENUMERIZE(prefix, elem, ...)         prefix ## _ ## elem
#define STRINGIFY(_, elem, ...)         #elem
#define MAP_TO_ONE(_, elem, ...)        1
#define MAP_TO_ENUM_CLASS(enum_class, elem, ...)  enum_class##::##elem

#define DECLARE_VARIABLES(uppercase, lowercase, camelcase) \
    enum camelcase \
    { \
       uppercase(ENUMERIZE, lowercase, COMMA), \
    }; \
    constexpr const char* lowercase##_names[]{ uppercase(STRINGIFY, EMPTY, COMMA), }; \
    constexpr size_t num_of_##lowercase##s = uppercase(MAP_TO_ONE, EMPTY, PLUS); \
    constexpr camelcase lowercase##_array[]{ uppercase(ENUMERIZE, lowercase, COMMA), }

/* a list of all possible detectors */
#define DETECTORS(action, arg, sep)       \
        action(arg, SHITOMASI)        sep \
        action(arg, HARRIS)           sep \
        action(arg, FAST)             sep \
        action(arg, BRISK)            sep \
        action(arg, ORB)              sep \
        action(arg, AKAZE)            sep \
        action(arg, SIFT)
DECLARE_VARIABLES(DETECTORS, detector, Detector);


/* a list of all possible descriptors */
#define DESCRIPTORS(action, arg, sep)        \
        action(arg, BRISK)               sep \
        action(arg, BRIEF)               sep \
        action(arg, ORB)                 sep \
        action(arg, FREAK)               sep \
        action(arg, AKAZE)               sep \
        action(arg, SIFT)
DECLARE_VARIABLES(DESCRIPTORS, descriptor, Descriptor);


/* a list of all possible descriptor types */
#define DESCRIPTOR_TYPES(action, arg, sep)        \
        action(arg, DES_BINARY)               sep \
        action(arg, DES_HOG)
DECLARE_VARIABLES(DESCRIPTOR_TYPES, descriptor_type, DescriptorType);


/* a list of all possible matchers */
#define MATCHERS(action, arg, sep)        \
        action(arg, MAT_BF)           sep \
        action(arg, MAT_FLANN)
DECLARE_VARIABLES(MATCHERS, matcher, Matcher);


/* a list of all possible selectors */
#define SELECTORS(action, arg, sep)        \
        action(arg, SEL_NN)           sep \
        action(arg, SEL_KNN)
DECLARE_VARIABLES(SELECTORS, selector, Selector);

inline std::vector<DescriptorType> CompatibleDescriptorTypes(const Descriptor descriptor)
{
  switch (descriptor)
  {
    case descriptor_BRISK:
    case descriptor_BRIEF:
    case descriptor_ORB:
    case descriptor_FREAK:
    case descriptor_AKAZE:
    case descriptor_SIFT:
      return { descriptor_type_DES_BINARY, descriptor_type_DES_HOG, };
    default:
      throw std::logic_error("some descriptors are not presented in the list of 'case' statements");
  }
}

template <typename T>
inline const char* ToString(const char* const names[], T index)
{
  return names[static_cast<size_t>(index)];
}

inline std::string ToString(Detector det)
{
  return ToString(detector_names, det);
}

inline std::string ToString(Descriptor desc)
{
  return ToString(descriptor_names, desc);
}

inline std::string ToString(DescriptorType desc_type)
{
  return ToString(descriptor_type_names, desc_type);
}

inline std::string ToString(Matcher mtch)
{
  return ToString(matcher_names, mtch);
}

inline std::string ToString(Selector sel)
{
  return ToString(selector_names, sel);
}

#endif /* dataStructures_h */
