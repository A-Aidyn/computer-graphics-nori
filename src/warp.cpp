/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

#define MyEpsilon 1e-6f

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    float new_x = r * cos(theta);
    float new_y = r * sin(theta);
    return Point2f(new_x, new_y);
    // throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    // float r = sqrt(p.x());
    // float theta = 2 * M_PI * p.y();
    if(p.dot(p) <= 1.0)
        return 1.0 / M_PI;
    else
        return 0.0;
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    float h = 1.0 - cosThetaMax;
    float new_z = 1.0 - h * sample.x();
    float r = sqrt(1.0 - new_z * new_z);
    float phi = 2.0 * M_PI * sample.y();
    float new_x = r * cos(phi);
    float new_y = r * sin(phi);
    return Vector3f(new_x, new_y, new_z);
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
    float h = 1.0 - cosThetaMax;
    float a = sqrt(1.0 - cosThetaMax * cosThetaMax);
    if(fabs(1.0 - v.dot(v)) <= MyEpsilon && (v.z() - cosThetaMax) >= MyEpsilon)
        return 1.0 / (M_PI * (a * a + h * h));
    else
        return 0.0;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float new_z = 2.0 * sample.x() - 1.0;
    float r = sqrt(1.0 - new_z * new_z);
    float phi = 2.0 * M_PI * sample.y();
    float new_x = r * cos(phi);
    float new_y = r * sin(phi);
    return Vector3f(new_x, new_y, new_z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if(fabs(1.0 - v.dot(v)) <= MyEpsilon)
        return 1.0 / (4.0 * M_PI);
    else
        return 0.0;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float new_z = sample.x();
    float r = sqrt(1.0 - new_z * new_z);
    float phi = 2.0 * M_PI * sample.y();
    float new_x = r * cos(phi);
    float new_y = r * sin(phi);
    return Vector3f(new_x, new_y, new_z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if(fabs(1.0 - v.dot(v)) <= MyEpsilon && v.z() >= 0.0)
        return 1.0 / (2.0 * M_PI);
    else
        return 0.0;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    // Credits to John Hughes. Link: https://math.stackexchange.com/questions/1729012/mapping-the-unit-disc-to-the-hemisphere
    Point2f disk = squareToUniformDisk(sample);
    float new_z = sqrt(1.0 - disk.dot(disk));
    float new_x = disk.x(); 
    float new_y = disk.y();
    return Vector3f(new_x, new_y, new_z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if(fabs(1.0 - v.dot(v)) <= MyEpsilon && v.z() >= 0.0)
        return v.z() / (M_PI);
    else
        return 0.0;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float theta = atan(sqrt(-1.0 * alpha * alpha * log(sample.x())));
    float phi = 2 * M_PI * sample.y();

    float new_x = sin(theta) * cos(phi);
    float new_y = sin(theta) * sin(phi);
    float new_z = cos(theta);
    
    return Vector3f(new_x, new_y, new_z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if(fabs(1.0 - m.dot(m)) <= MyEpsilon && m.z() >= 0.0) {
        float cosTheta = m.z();
        float theta = acos(cosTheta);
        float pdf_upper = exp((-1.0 * tan(theta) * tan(theta)) / (alpha * alpha));
        float pdf_lower = M_PI * (alpha * alpha) * cosTheta * cosTheta * cosTheta;
        return pdf_upper / pdf_lower;
    } else
        return 0.0;
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

NORI_NAMESPACE_END
