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

class Plastic : public BSDF {
public:
    Plastic(const PropertyList &propList) {
        /* RMS surface roughness */
        m_roughness = propList.getFloat("roughness", 0.1f);

        float log_r = log(m_roughness);
        m_alpha = 1.62142f + 0.819955f * log_r + 0.1734f * log_r * log_r + 0.0171201f * log_r * log_r * log_r + 0.000640711f * log_r * log_r * log_r * log_r;

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.0f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.25f));
        m_ks = propList.getColor("ks", Color3f(0.25f));

        m_coef = propList.getFloat("coef", 0.5f);

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        // m_ks = 1 - m_kd.maxCoeff();
    }

    float evalTrowbridgeReitz(const Normal3f &m) const {
        float tan2Theta = Frame::tanTheta(m) * Frame::tanTheta(m);
        float ct = Frame::cosTheta(m);
        float ct4 = ct * ct * ct * ct;
        float e = (1.0f / (m_alpha * m_alpha)) * tan2Theta;
        return 1 / (M_PI * m_alpha * m_alpha * ct4 * (1 + e) * (1 + e));
    }

    float trowbridgeReitzG1(const Vector3f &v, const Normal3f &m) const {
        float tanTheta = Frame::tanTheta(v);

        /* Perpendicular incidence -- no shadowing/masking */
        // if (tanTheta == 0.0f)
        //     return 1.0f;

        // /* Can't see the back side from the front and vice versa */
        // if (m.dot(v) * Frame::cosTheta(v) <= 0)
        //     return 0.0f;

        float alpha2Tan2Theta = (m_alpha * tanTheta) * (m_alpha * tanTheta);

        return 2.0f / (1.0f + sqrt(1.0f + alpha2Tan2Theta));
    }

    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
    	Color3f result(0.0);
        if (bRec.measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
            return result;
        Vector3f w_h = (bRec.wi + bRec.wo).normalized();
        float D = evalTrowbridgeReitz(w_h);
        float G = trowbridgeReitzG1(bRec.wi, w_h) * trowbridgeReitzG1(bRec.wo, w_h);
        float F = fresnel(w_h.dot(bRec.wi), m_extIOR, m_intIOR);
        result = (m_kd * INV_PI) + m_ks * (D * F * G) / (4.0 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo));
        return result;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        if (bRec.measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;
        Vector3f w_h = (bRec.wi + bRec.wo).normalized();
        float J_h = 1.0 / (4.0 * w_h.dot(bRec.wo));
        return (evalTrowbridgeReitz(w_h) * Frame::cosTheta(w_h) * J_h + INV_PI * Frame::cosTheta(bRec.wo)) * m_coef;
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);
        // need to generate bRec.wo
        bRec.measure = ESolidAngle;
        bRec.eta = 1.0f;
        if(_sample.x() < m_coef) { // beckmann
            Point2f newSample(_sample.x() / m_coef, _sample.y());
            Vector3f w_h = Warp::squareToTrowbridgeReitz(newSample, m_alpha).normalized();
            // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#Reflect
            bRec.wo = (-bRec.wi + 2.0 * bRec.wi.dot(w_h) * w_h).normalized(); 
        } else {  // cosine-weighted
            Point2f newSample( (_sample.x() / m_coef - 1.0) / (1.0 / m_coef - 1.0), _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(newSample);
        }
        if(pdf(bRec) <= 0.0f)
            return Color3f(0.0f);
        return (eval(bRec) / pdf(bRec)) * Frame::cosTheta(bRec.wo);    
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Plastic[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_coef;
    float m_roughness;
    float m_alpha;
    float m_intIOR, m_extIOR;
    Color3f m_kd, m_ks;
};

NORI_REGISTER_CLASS(Plastic, "plastic");
NORI_NAMESPACE_END
