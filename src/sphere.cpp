/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

bool bigger(float a, float b) {  // returns true if a > b
    if( (a - b) > Epsilon)
        return true;
    return false;
}

class Sphere : public Shape {
public:
    Sphere(const PropertyList & propList) {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {
        Vector3f ray_to_sphere = (m_position - ray.o);
        float tc = ray_to_sphere.dot(ray.d);

        if(tc < 0)
            return false;
        
        float d = sqrt(ray_to_sphere.dot(ray_to_sphere) - tc * tc);
        if(d > m_radius) return false;
        float t1c = sqrt(m_radius * m_radius - d * d);
        
        float t1 = tc - t1c;
        float t2 = tc + t1c;

        bool ok1 = false;
        bool ok2 = false;

        if(ray.mint < t1 && t1 < ray.maxt)
            ok1 = true;
        if(ray.mint < t2 && t2 < ray.maxt)
            ok2 = true;

        if(ok1 && ok2)
            t = std::min(t1, t2);
        else if(ok1)
            t = t1;
        else if(ok2)
            t = t2;
        else 
            return false;
        
        return true;        
        /*
        Vector3f om = Vector3f(ray.o - m_position);
        auto A = ray.d.dot(ray.d);
        auto B = 2 * ray.d.dot(om);
        auto C = om.dot(om) - m_radius * m_radius;

        auto D = B * B - 4 * A * C;
        if(D < 0.0) { // negative
            return false;
        }

        auto t1 = (-B + sqrt(D)) / (2.0 * A);
        auto t2 = (-B - sqrt(D)) / (2.0 * A);

        bool ok1 = false, ok2 = false;

        if(bigger(t1, ray.mint) && bigger(ray.maxt, t1))
            ok1 = true;
        
        if(bigger(t2, ray.mint) && bigger(ray.maxt, t2))
            ok2 = true;    

        if(ok1 && ok2)
            t = std::min(t1, t2);
        else if(ok1)
            t = t1;
        else if(ok2)
            t = t2;

        if(ok1 && ok2)
            cout << "t1: " << t1 << " t2: " << t2 << endl;

        return (ok1 | ok2); */
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const override {
        /* to be implemented */
        // float u = 0, v = 0, t = 0;
        // this->rayIntersect(index, ray, u, v, t);
        Point3f intersectionPoint = ray.o + ray.d * ray.maxt;
        Vector3f sctip = intersectionPoint - m_position;
        Point2f uv = sphericalCoordinates(sctip.normalized());
        its.p = intersectionPoint;
        // its.uv = Point2f(uv.x() / M_PI, uv.y() / (2.0 * M_PI));
        // its.uv = uv;
        its.uv = Point2f(uv.y() / (2.0 * M_PI), uv.x() / M_PI);
        its.shFrame = Frame(sctip.normalized());
        its.geoFrame = Frame(sctip.normalized());
    }

    virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }


    virtual std::string toString() const override {
        return tfm::format(
                "Sphere[\n"
                "  center = %s,\n"
                "  radius = %f,\n"
                "  bsdf = %s,\n"
                "  emitter = %s\n"
                "]",
                m_position.toString(),
                m_radius,
                m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
                m_emitter ? indent(m_emitter->toString()) : std::string("null"));
    }

protected:
    Point3f m_position;
    float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
