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

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
        // m_emittedPhoton = 0;
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;	

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here
        auto lights = scene -> getLights();
        int nLights = lights.size();        
        std::vector<std::pair<std::pair<Point3f, Vector3f>, Color3f> > tmp[nLights];
        int emitted[nLights];
        memset(emitted, 0, sizeof emitted);

        int deposited = 0;

        // cout << "n"

        while(deposited < m_photonCount) {
            int selectedLight = int((sampler -> next1D()) * nLights) % nLights;
            auto light = lights[selectedLight];
            Ray3f ray;
            light -> samplePhoton(ray, sampler -> next2D(), sampler -> next2D());
            Color3f throughput = 1.0f;
            emitted[selectedLight] ++;
            // m_emittedPhoton ++;
            // cout << "deposited: " << deposited << endl;
            while(deposited < m_photonCount) {
                Intersection its;
                if(!scene->rayIntersect(ray, its))
                    break;
                if(!its.mesh)
                    break;
                Vector3f w = (ray.o - its.p).normalized();
                BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());
                if(bsdf && bsdf -> isDiffuse()) {
                    tmp[selectedLight].push_back(std::make_pair(std::make_pair(its.p, w), throughput));
                    deposited ++;
                    // m_photonMap -> push_back(Photon(
                    //     its.p,
                    //     w,
                    //     power * throughput
                    // ));
                }
                if(bsdf) {
                    Vector3f w_local = its.toLocal(w);
                    BSDFQueryRecord bRec(w_local);  
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    Point2f bsdfSample = sampler -> next2D();
                    Color3f bsdfResult = bsdf -> sample(bRec, bsdfSample);
                    // bsdf -> sample(bRec, bsdfSample);

                    // if(bsdfSampleResult.maxCoeff() == 0.0f)
                        // break;
                    Vector3f w_sampled = its.toWorld(bRec.wo).normalized(); // sampled direction 
                    
                    // BSDFQueryRecord bRec2(its.toLocal(w_sampled), its.toLocal(w), ESolidAngle);
                    // bRec2.uv = its.uv;
                    // bRec2.p = its.p;                    
                    // if((bsdf -> pdf(bRec2)) > 0.0f) {
                    //     bsdfResult = ((bsdf -> eval(bRec2)) / (bsdf -> pdf(bRec2))) * Frame::cosTheta(bRec2.wo);
                    // }
                    
                    Ray3f ray_w_sampled(its.p, w_sampled);
                    float successProb = std::min(throughput.maxCoeff(), 0.99f);                                    
                    if((sampler -> next1D()) <= successProb) {
                        throughput /= successProb;
                        throughput *= bsdfResult;                    
                        ray = ray_w_sampled;
                    } else 
                        break;
                } else {
                    break;
                }
            }
        }
        // cout << "first part finished!" << endl;
        for(int i = 0; i < nLights; i ++) {
            auto light = lights[i];
            Ray3f ray;
            Color3f power = light -> samplePhoton(ray, sampler -> next2D(), sampler -> next2D());
            for(auto& cur : tmp[i]) {
                m_photonMap -> push_back(Photon(
                    cur.first.first,
                    cur.first.second,
                    (cur.second * power) / (1.0f * emitted[i])  // cur.second: throughput                        
                ));
            }
        }

		/* Build the photon map */
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// put your code for path tracing with photon gathering here

        Color3f L(0.0f);
        Ray3f ray(_ray);
        Color3f throughput(1.0f);
        while(true) {
            Intersection its;
            if(!scene->rayIntersect(ray, its))
                break;
            if(!its.mesh)
                break;

            if(its.mesh -> isEmitter()) {
                EmitterQueryRecord lRec (ray.o, its.p, its.shFrame.n);
                if(its.mesh -> getEmitter() -> pdf(lRec) > 0.0)
                    L += its.mesh -> getEmitter() -> eval(lRec) * throughput;
            }

            BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());
            if(bsdf && bsdf -> isDiffuse()) {
                // calculate photons
                std::vector<uint32_t> results; 
                m_photonMap->search(its.p, m_photonRadius, results);

                Color3f curResult(0.0f);
                for(auto i : results) {
                    const Photon &photon = (*m_photonMap)[i];
                    Vector3f wo = photon.getDirection();
                    Vector3f wi = (ray.o - its.p).normalized();
                    BSDFQueryRecord bRec(its.toLocal(wi), its.toLocal(wo), ESolidAngle);
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    if(bsdf -> pdf(bRec) != 0.0f) {
                        Color3f bsdfResult = ((bsdf -> eval(bRec)) / (bsdf -> pdf(bRec))) * Frame::cosTheta(bRec.wo);
                        curResult += (bsdfResult * photon.getPower());                    
                    }
                }
                curResult = curResult / ((M_PI * M_PI * m_photonRadius * m_photonRadius));
                L += throughput * curResult;
                break;
            }

            if(bsdf) {
                Vector3f w = (ray.o - its.p).normalized();
                Vector3f w_local = its.toLocal(w);
                BSDFQueryRecord bRec(w_local);  
                bRec.uv = its.uv;
                bRec.p = its.p;
                Point2f bsdfSample = sampler -> next2D();
                Color3f bsdfResult = bsdf -> sample(bRec, bsdfSample);
                Vector3f w_sampled = its.toWorld(bRec.wo).normalized(); // sampled direction 
                Ray3f ray_w_sampled(its.p, w_sampled);
                float successProb = std::min(throughput.maxCoeff(), 0.99f);                                    
                if((sampler -> next1D()) <= successProb) {
                    throughput /= successProb;
                    throughput *= bsdfResult;                    
                    ray = ray_w_sampled;
                } else 
                    break;
            } else {
                break;
            }
        }
		return L;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */ 
    // int m_emittedPhoton;
    int m_photonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
