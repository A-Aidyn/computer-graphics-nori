#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PathMats : public Integrator {
public:
    PathMats(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f L(0.0);
        Ray3f curRay = ray;
        Color3f throughput(1.0f);
        while(true) {
            Intersection its;
            // cout << "hello?\n";
            if(!scene->rayIntersect(curRay, its))
                break;
            if(!its.mesh)
                break;

            if(its.mesh -> isEmitter()) {
                EmitterQueryRecord lRec (curRay.o, its.p, its.shFrame.n);
                if(its.mesh -> getEmitter() -> pdf(lRec) > 0.0)
                    L += its.mesh -> getEmitter() -> eval(lRec) * throughput;
            }

            if(its.mesh -> getBSDF()) {
                BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());

                Vector3f w = (curRay.o - its.p).normalized();
                Vector3f w_local = its.toLocal(w);
                BSDFQueryRecord bRec(w_local);                
                bRec.uv = its.uv;
                bRec.p = its.p;
                Point2f bsdfSample = sampler -> next2D();
                Color3f bsdfResult = bsdf -> sample(bRec, bsdfSample);

                Vector3f w_sampled = its.toWorld(bRec.wo).normalized(); // sampled direction 

                Ray3f ray_w_sampled(its.p, w_sampled);

                float successProb = std::min(throughput.maxCoeff(), 0.99f);                                    
                if((sampler -> next2D()).x() <= successProb) {
                    throughput /= successProb;
                    throughput *= bsdfResult;                    
                    curRay = ray_w_sampled;
                    // L += Li(scene, sampler, ray_w_sampled) * bsdfResult / (1 - successProb);  // multiplication by cosine is handled in bsdf
                } else 
                    break;
            }
        }
        
        return L;
    }

    std::string toString() const {
        return tfm::format("PathMats[]");
    }
};

NORI_REGISTER_CLASS(PathMats, "path_mats");
NORI_NAMESPACE_END