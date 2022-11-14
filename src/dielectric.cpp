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
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    virtual Color3f eval(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        float F = fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);
        bRec.measure = EDiscrete;

        // if(Frame::cosTheta(bRec.wi) < 0) {
        //     cout << "F: " << F << " sample.x(): " << sample.x() << " cosTheta: " << Frame::cosTheta(bRec.wi) << endl;
        // }
        
        if(sample.x() < F) { // reflection
            bRec.wo = Vector3f(
                -bRec.wi.x(),
                -bRec.wi.y(),
                 bRec.wi.z()
            );
            // return F * Color3f(1.0f);
        } else { // refraction
            Vector3f n(0, 0, 1);
            if(Frame::cosTheta(bRec.wi) < 0) {
                // std::cout << "entered2?";
                n = -n;
            }

            bool entering = Frame::cosTheta(bRec.wi) > 0;

            float fromIOR = (entering) ? m_extIOR : m_intIOR; 
            float toIOR = (entering) ? m_intIOR : m_extIOR;            

            float fromIOR2 = fromIOR * fromIOR;
            float toIOR2 = toIOR * toIOR;

            float cosThetaI = n.dot(bRec.wi);
            float sin2I = std::max(0.0f, 1.0f - cosThetaI * cosThetaI); // sin^2(theta)_incident
            float sin2T = sin2I * fromIOR2 / toIOR2; // sin^2(theta)_transmitted

            if(sin2T >= 1.0) return Color3f(0.0f);

            float cosT = sqrt(1.0 - sin2T);

            float fracIOR = fromIOR / toIOR; 
            bRec.wo = fracIOR * -bRec.wi + (fracIOR * cosThetaI - cosT) * n;
            // bRec.wo = - fracIOR * (bRec.wi - Frame::cosTheta(bRec.wi) * n) - n * sqrt(1.0 - fracIOR * fracIOR * (1.0 - Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wi)));
            // bRec.wo = bRec.wo.normalized();
            
            // cout << "cosTheta(bRec.wo): " << Frame::cosTheta(bRec.wo) << endl;

            
            // return (1.0 - F) * Color3f(1.0f);
        }
        return Color3f(1.0f);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
