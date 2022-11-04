#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLightMats : public Integrator {
public:
    DirectLightMats(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f L(0.0);
        Intersection its;
        if(!scene->rayIntersect(ray, its))
            return L;
        
        if(!its.mesh)
            return L;

        if(its.mesh -> isEmitter()) {
            EmitterQueryRecord lRec (ray.o, its.p, its.shFrame.n);
            if(its.mesh -> getEmitter() -> pdf(lRec) > 0.0)
                L += its.mesh -> getEmitter() -> eval(lRec);
        }

        if(its.mesh -> getBSDF()) {
            BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());

            Vector3f w = (ray.o - its.p).normalized();

            // std::cout << "hello\n";
            for(auto light : scene -> getLights()) {
                // std::cout << "bsdfResult: " << bsdfResult.toString() << endl;

                // std::cout << "shading normal: " << its.shFrame.n.toString() << endl;
                // std::cout << "wi: " << lRec.wi.toString() << endl;
            
                BSDFQueryRecord bRec(its.toLocal(w));                
                bRec.uv = its.uv;
                bRec.p = its.p;
                Point2f bsdfSample = sampler -> next2D();
                Color3f bsdfResult = bsdf -> sample(bRec, bsdfSample);

                // std::cout << "bRec.wi: " << bRec.wi.toString() << " | bRec.wo: " << bRec.wo.toString() << endl;

                Vector3f w_sampled = its.toWorld(bRec.wo).normalized(); // sampled direction 
                float cosine = 1.0; // fabs(its.shFrame.n.dot(w_sampled));


                Ray3f ray_w_sampled;
                ray_w_sampled.o = its.p;
                ray_w_sampled.d = w_sampled;
                ray_w_sampled.update();

                Intersection new_its;

                // std::cout << ray_w_sampled.maxt << endl;

                if(!scene->rayIntersect(ray_w_sampled, new_its))
                    continue;

                if(!new_its.mesh)
                    continue;

                if(new_its.mesh -> isEmitter()) {
                    if(new_its.mesh -> getEmitter() == light) {
                        EmitterQueryRecord lRec (its.p, new_its.p, new_its.shFrame.n);
                        Color3f lightResult = new_its.mesh -> getEmitter() -> eval(lRec);
                        L += (lightResult * bsdfResult * cosine);
                    }
                }
            }
        }


        return L;
    }

    std::string toString() const {
        return tfm::format("DirectLightMats[]");
    }
};

NORI_REGISTER_CLASS(DirectLightMats, "direct_mats");
NORI_NAMESPACE_END