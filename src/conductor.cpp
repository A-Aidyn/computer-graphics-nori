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

#include <nori/bsdf.h>
#include <nori/common.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Conductor : public BSDF {
public:
    Conductor(const PropertyList &propList) {
        m_specular_reflectance = propList.getColor("specularReflectance", Color3f(1.0f));
        m_eta = propList.getColor("eta", Color3f(0.0f));
        m_k = propList.getColor("k", Color3f(1.0f));
    }

    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
    	/* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f fresnelConductor(float cosThetaI, Color3f extIOR, Color3f intIOR, Color3f k) const {
        float cosThetaI2 = cosThetaI * cosThetaI;
        float sinThetaI2 = 1.0f - cosThetaI2;
        Color3f eta2 = (intIOR / extIOR) * (intIOR / extIOR);
        Color3f etak2 = (k / extIOR) * (k / extIOR);

        Color3f t0 = eta2 - etak2 - sinThetaI2;
        Color3f a2plusb2 = sqrt(t0 * t0 + 4 * eta2 * etak2);
        Color3f t1 = a2plusb2 + cosThetaI2;
        Color3f a = sqrt(0.5f * (a2plusb2 + t0));
        Color3f t2 = 2.0f * cosThetaI * a;
        Color3f Rs = (t1 - t2) / (t1 + t2);

        Color3f t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
        Color3f t4 = t2 * sinThetaI2;
        Color3f Rp = Rs * (t3 - t4) / (t3 + t4);

        return (Rp + Rs) / 2.0f;
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);
        bRec.wo = Vector3f(
            -bRec.wi.x(),
            -bRec.wi.y(),
             bRec.wi.z()
        );
        bRec.measure = EDiscrete;
        bRec.eta = 1.0f;
        return m_specular_reflectance * fresnelConductor(Frame::cosTheta(bRec.wi), 1.0f, m_eta, m_k);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Conductor[\n"
            "  m_specular_reflectance %s,\n"
            "  m_eta %s,\n"
            "  m_k = %s\n"
            "]",
            m_specular_reflectance.toString(),
            m_eta.toString(),
            m_k.toString()
        );
    }
private:
    Color3f m_specular_reflectance;
    Color3f m_eta, m_k;
};

NORI_REGISTER_CLASS(Conductor, "conductor");
NORI_NAMESPACE_END
