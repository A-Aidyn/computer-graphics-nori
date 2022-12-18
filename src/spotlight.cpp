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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class SpotLight : public Emitter {
public:
    SpotLight(const PropertyList &props) {
        power = props.getColor("power");
        position = props.getPoint3("position");
        cosTotalWidth = props.getFloat("cosTotalWidth");
        cosFalloffStart = props.getFloat("cosFalloffStart");
    }

    float falloff(const Vector3f &w) const {
        float cosTheta = w.dot(Vector3f(0.0f, -1.0f, 0.0f));
        if(cosTheta > cosFalloffStart)
            return 1.0f;
        if(cosTheta < cosTotalWidth) 
            return 0.0f;
        float delta = (cosTheta - cosTotalWidth) / (cosFalloffStart - cosTotalWidth);
        return delta * delta * delta * delta;
    }

    // Returns irradiance at the given point
    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        lRec.wi = (position - lRec.ref).normalized();
        lRec.pdf = 0.0;
        lRec.p = position;
        float sqrdst = (position - lRec.ref).dot(position - lRec.ref);
        return (this -> eval(lRec) * falloff(-lRec.wi)) / sqrdst;
    }

    // Returns irradiance of the light source
    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        return power / (2.0 * M_PI * (1.0f - 0.5f * (cosTotalWidth + cosFalloffStart)));
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        return 0.0;
    }

    // virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
    //     throw NoriException("To implement...");
    // }

    virtual std::string toString() const override {
        return tfm::format(
                "SpotLight[\n"
                "  power = %s,\n"
                "  position = %s \n"
                "]",
                power.toString(),
                position.toString());
    }

protected:
    Color3f power;
    Point3f position;
    float cosTotalWidth, cosFalloffStart;
};

NORI_REGISTER_CLASS(SpotLight, "spot")
NORI_NAMESPACE_END