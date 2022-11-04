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

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        float sqrdst = (lRec.p - lRec.ref).dot(lRec.p - lRec.ref);
        Vector3f negwi = -lRec.wi;
        if(lRec.n.dot(negwi) > 0.0)
            return m_radiance;
        return Color3f(0.0);
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        ShapeQueryRecord sRec(lRec.ref);
        m_shape -> sampleSurface(sRec, sample);

        lRec.p = sRec.p;
        lRec.n = sRec.n;
        lRec.pdf = sRec.pdf;
        lRec.wi = (sRec.p - lRec.ref).normalized();
        float solid_angle_pdf = sRec.pdf * (lRec.p - lRec.ref).dot((lRec.p - lRec.ref)) / fabs(lRec.n.dot(-lRec.wi));
        return eval(lRec) / solid_angle_pdf;
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        ShapeQueryRecord sRec(lRec.ref);
        float pdf = m_shape -> pdfSurface(sRec);
        // if(pdf == 0.0) return pdf;
        float solid_angle_pdf = pdf * (lRec.p - lRec.ref).dot((lRec.p - lRec.ref)) / fabs(lRec.n.dot(-lRec.wi));
        return pdf;
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        throw NoriException("To implement...");
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END