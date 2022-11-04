#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLightMis : public Integrator {
public:
    DirectLightMis(const PropertyList &props) {
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
            Color3f lightResult = its.mesh -> getEmitter() -> eval(lRec);
            L += lightResult;
        }

        if(its.mesh -> getBSDF()) {
            // std::cout << "hello\n";
            BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());
            Vector3f w = (ray.o - its.p).normalized();

            for(auto light : scene -> getLights()) {
                Color3f ems_value(0.0f);
                float ems_pdf1 = 0.0f;
                float mats_pdf1 = 0.0f;

                EmitterQueryRecord lRec (its.p);
                Point2f sample = sampler -> next2D();
                Color3f lightResult = light -> sample(lRec, sample);
                
                Ray3f fromPointToLight;
                fromPointToLight.o = its.p;
                fromPointToLight.d = lRec.wi;
                fromPointToLight.maxt = sqrt((lRec.p - lRec.ref).dot((lRec.p - lRec.ref))) - Epsilon;
                fromPointToLight.update();

                Intersection newIts;

                if (!scene->rayIntersect(fromPointToLight, newIts)) {
                    Vector3f wo = (ray.o - lRec.ref).normalized();
                    BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(wo), ESolidAngle);
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    Color3f bsdfResult = bsdf -> eval(bRec);
                    float cosine = fabs(its.shFrame.n.dot(lRec.wi));

                    ems_value = (lightResult * bsdfResult * cosine);
                    ems_pdf1 = light -> pdf(lRec);
                    mats_pdf1 = bsdf -> pdf(bRec);

                    if(ems_pdf1 > 0.0f) {
                        if(mats_pdf1 <= Epsilon) {
                            L += ems_value;
                        } else {
                            float total_pdf = ems_pdf1 + mats_pdf1;
                            float weight = ems_pdf1 / total_pdf;
                            L += (ems_value * weight);
                        }
                    }
                }
                
                // --------------------------------------
                // --- BSDF  --
                // -------------------

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

                Ray3f ray_w_sampled;
                ray_w_sampled.o = its.p;
                ray_w_sampled.d = w_sampled;
                ray_w_sampled.update();

                Intersection new_its;

                if(scene->rayIntersect(ray_w_sampled, new_its)) {
                    if(new_its.mesh) {
                        if(new_its.mesh -> isEmitter()) {
                            if(new_its.mesh -> getEmitter() == light) {
                                EmitterQueryRecord lRec2 (its.p, new_its.p, new_its.shFrame.n);
                                Color3f lightResult2 = light -> eval(lRec2);
                                mats_value = (lightResult2 * bsdfResult2);
                                mats_pdf2 = bsdf -> pdf(bRec2);
                                ems_pdf2 = light -> pdf(lRec2);


                                if(mats_pdf2 > 0.0f) {
                                    if(ems_pdf2 <= Epsilon) {
                                        L += (mats_value);
                                    } else {
                                        float total_pdf = ems_pdf2 + mats_pdf2;
                                        float weight = mats_pdf2 / total_pdf;
                                        L += (mats_value * weight);
                                    }
                                }
                            }
                        }
                    }
                }

                // std::cout << "diff ems_pdf: " << fabs(ems_pdf1 - ems_pdf2) << " diff mats_pdf: " << fabs(mats_pdf1 - mats_pdf2) << endl;

            }
        }


        return L;
    }

    std::string toString() const {
        return tfm::format("DirectLightMis[]");
    }
};

NORI_REGISTER_CLASS(DirectLightMis, "direct_mis");
NORI_NAMESPACE_END