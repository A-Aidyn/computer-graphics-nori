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

class PointLight : public Emitter {
public:
    PointLight(const PropertyList &props) {
        power = props.getColor("power");
        position = props.getPoint3("position");
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        lRec.wi = (position - lRec.ref).normalized();
        lRec.pdf = 1.0;
        lRec.p = position;
        return (this -> eval(lRec));
    }

    // Returns irradiance at the given point
    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        float sqrdst = (position - lRec.ref).dot(position - lRec.ref);
        return power / (4.0 * M_PI) / (sqrdst);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        return 1.0;
    }

    // virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
    //     throw NoriException("To implement...");
    // }

    virtual std::string toString() const override {
        return tfm::format(
                "PointLight[\n"
                "  power = %s,\n"
                "  position = %s \n"
                "]",
                power.toString(),
                position.toString());
    }

protected:
    Color3f power;
    Point3f position;
};

NORI_REGISTER_CLASS(PointLight, "point")
NORI_NAMESPACE_END