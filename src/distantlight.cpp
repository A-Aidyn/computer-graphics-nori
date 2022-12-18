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

class DistantLight : public Emitter {
public:
    DistantLight(const PropertyList &props) {
        power = props.getColor("power");
        lightToPointDirection = props.getVector3("lightToPointDirection");
        worldRadius = props.getFloat("worldRadius", 1000.0f);
    }

    // Returns irradiance at the given point
    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        lRec.wi = -lightToPointDirection.normalized();
        lRec.pdf = 0.0;
        lRec.p = lRec.ref - 2 * worldRadius * lightToPointDirection;
        return this -> eval(lRec);
    }

    // Returns irradiance of the light source
    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        return power / (M_PI * worldRadius * worldRadius);    
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        return 1.0;
    }

    // virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
    //     throw NoriException("To implement...");
    // }

    virtual std::string toString() const override {
        return tfm::format(
                "DistantLight[\n"
                "  power = %s,\n"
                "  lightToPointDirection = %s \n"
                "]",
                power.toString(),
                lightToPointDirection.toString());
    }

protected:
    Color3f power;
    Vector3f lightToPointDirection;
    float worldRadius;
};

NORI_REGISTER_CLASS(DistantLight, "distant")
NORI_NAMESPACE_END