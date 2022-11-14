#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PathMis : public Integrator {
    float q = 0.1;

    Color3f calculateWeightedValue(Color3f value, float a, float b) const {
        if(a > 0.0f) {
            if(b <= Epsilon)
                return value;
            // cout << "lolkek\n";
            float total = a + b;
            float weight = a / total;
            return value * weight;
        }
        return Color3f(0.0f);
    }

    Color3f RecLi(const Scene *scene, Sampler *sampler, const Ray3f &ray, Color3f throughput, int bounces = 0, bool specular = false, float oldBsdfpdf = 0.0f) const {
        Color3f L(0.0);
        Intersection its;
        if(!scene->rayIntersect(ray, its))
            return L;
        
        if(!its.mesh)
            return L;

        if(its.mesh -> isEmitter()) {
            EmitterQueryRecord lRec (ray.o, its.p, its.shFrame.n);
            auto light = its.mesh -> getEmitter();
            if(light -> pdf(lRec) > 0.0) {
                if(!bounces)
                    L += throughput * (light -> eval(lRec));
                else {
                    Color3f mats_value = throughput * (light -> eval(lRec));
                    float mats_pdf = oldBsdfpdf;
                    float ems_pdf = light -> pdf(lRec);
                    if(specular)
                        mats_pdf = 1.0f, ems_pdf = 0.0f;
                    L += calculateWeightedValue(mats_value, mats_pdf, ems_pdf);
                }
            }
        }

        if(its.mesh -> getBSDF()) {
            // std::cout << "hello\n";
            BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());
            Vector3f w = (ray.o - its.p).normalized();
            float ems_pdf = 0.0f;
            float mats_pdf = 0.0f;

            auto lights = scene -> getLights();
            int nLights = lights.size();
            for(auto light : lights) {
                // int selectedLight = int((sampler -> next1D()) * nLights) % nLights;
                // auto light = lights[selectedLight];

                // -----------------------------
                // ------ Light sampling  ------
                // -----------------------------

                Color3f ems_value(0.0f);
                ems_pdf = 0.0f;
                mats_pdf = 0.0f;

                EmitterQueryRecord lRec (its.p);
                Point2f sample = sampler -> next2D();
                Color3f lightResult = light -> sample(lRec, sample);
                
                Ray3f fromPointToLight(its.p, lRec.wi, Epsilon, sqrt((lRec.p - lRec.ref).dot((lRec.p - lRec.ref))) - Epsilon);

                Intersection newIts;

                if (!scene->rayIntersect(fromPointToLight, newIts)) {
                    Vector3f wo = (ray.o - lRec.ref).normalized();
                    BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(wo), ESolidAngle);
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    Color3f bsdfResult = bsdf -> eval(bRec);
                    float cosine = fabs(its.shFrame.n.dot(lRec.wi));

                    ems_value = (lightResult * bsdfResult * cosine);
                    ems_pdf = light -> pdf(lRec);
                    mats_pdf = bsdf -> pdf(bRec);
                    if(bRec.measure != EDiscrete)
                    L += throughput * calculateWeightedValue(ems_value, ems_pdf, mats_pdf);
                }
            }

            // ----------------------------
            // ------ BSDF sampling  ------
            // ----------------------------

            Color3f mats_value(0.0f);
            float ems_pdf2 = 0.0f;
            float mats_pdf2 = 0.0f;

            BSDFQueryRecord bRec2(its.toLocal(w));                
            bRec2.uv = its.uv;
            bRec2.p = its.p;
            Point2f bsdfSample = sampler -> next2D();
            Color3f bsdfResult2 = bsdf -> sample(bRec2, bsdfSample);

            // std::cout << "bRec.wi: " << bRec.wi.toString() << " | bRec.wo: " << bRec.wo.toString() << endl;

            Vector3f w_sampled = its.toWorld(bRec2.wo).normalized(); // sampled direction 

            Ray3f ray_w_sampled(its.p, w_sampled);

            std::swap(bRec2.wi, bRec2.wo);
            float oldBsdfPdf = bsdf -> pdf(bRec2);

            // --------------------------------------
            // ------ Shooting new ray  -------------
            // --------------------------------------

            float successProb = std::min(throughput.maxCoeff(), 0.99f);
            // float successProb = 0.99f;
            if((sampler -> next1D()) <=  successProb && successProb > Epsilon) {
                L += RecLi(scene, sampler, ray_w_sampled, throughput * bsdfResult2 / successProb, bounces + 1, (bRec2.measure == EDiscrete), oldBsdfPdf);
            }
        }
        return L;
    }


public:
    PathMis(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        return RecLi(scene, sampler, ray, Color3f(1.0f));
    }

    std::string toString() const {
        return tfm::format("PathMis[]");
    }
};

NORI_REGISTER_CLASS(PathMis, "path_mis");
NORI_NAMESPACE_END