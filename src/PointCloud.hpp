// PointCloud.hpp
#ifndef __SLPOINTCLOUD_H__
#define __SLPOINTCLOUD_H__

#include <zed/utils/GlobalDefine.hpp>
#include "utils.hpp"

class PointCloud
{
public:

    PointCloud();
    PointCloud(size_t width, size_t height);
    virtual ~PointCloud();

    void fill(const unsigned char* image, const float* depth, const sl::zed::StereoParameters *param);

    POINT3D Point(size_t i, size_t j);

    size_t GetNbPoints();
    size_t GetWidth();
    size_t GetHeight();

	void WritePCDFile(std::string path, bool verbose = false);

    // Iterator definition
    typedef std::vector<POINT3D>::iterator iterator;
    typedef std::vector<POINT3D>::const_iterator const_iterator;

    iterator begin() {
        return pc.begin();
    }

    iterator end() {
        return pc.end();
    }

    const_iterator cbegin() {
        return pc.cbegin();
    }

    const_iterator cend() {
        return pc.cend();
    }

private:
    std::vector<POINT3D> pc;

    int Width;
    int Height;
};
#endif /* __SLPOINTCLOUD_H__ */
