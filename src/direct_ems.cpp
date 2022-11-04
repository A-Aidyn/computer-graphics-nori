#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLightEms : public Integrator {
public:
    DirectLightEms(const PropertyList &props) {
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

            for(auto light : scene -> getLights()) {
                EmitterQueryRecord lRec (its.p);
                Point2f sample = sampler -> next2D();
                Color3f lightResult = light -> sample(lRec, sample);

                // std::cout << "lightResult: " << lightResult.toString() << endl;

                Ray3f fromPointToLight;
                fromPointToLight.o = its.p;
                fromPointToLight.d = lRec.wi;
                fromPointToLight.maxt = sqrt((lRec.p - lRec.ref).dot((lRec.p - lRec.ref))) - Epsilon;
                fromPointToLight.update();

                Intersection newIts;

                if (scene->rayIntersect(fromPointToLight, newIts))
                    continue;

                Vector3f wo = (ray.o - lRec.ref).normalized();
                // Vector3f wo = (ray.o - lRec.ref);
                BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(wo), ESolidAngle);
                
                // bRec.uv = Point2f(its.uv.x() / M_PI, its.uv.y() / (2.0 * M_PI));
                // bRec.uv = Point2f(its.uv.y() / (2.0 * M_PI), its.uv.x() / M_PI);
                // std::cout << 1 / 0;
                // assert(0);
                // assert(its.uv.x() >= 0 && its.uv.x() <= 1);
                // assert(its.uv.y() >= 0 && its.uv.y() <= 1);
                bRec.uv = its.uv;
                // bRec.uv = Point2f(its.uv.y() / (2.0 * M_PI), its.uv.x() / M_PI);
                // Point2f uv = sphericalCoordinates(its.shFrame.n);
                // bRec.uv = Point2f(uv.y() / (2.0 * M_PI), uv.x() / M_PI);
                // bRec.uv = Point2f(uv.x() / M_PI, uv.y() / (2.0 * M_PI));

                bRec.p = its.p;
                Color3f bsdfResult = bsdf -> eval(bRec);

                // std::cout << "bsdfResult: " << bsdfResult.toString() << endl;

                // std::cout << "shading normal: " << its.shFrame.n.toString() << endl;
                // std::cout << "wi: " << lRec.wi.toString() << endl;
                

                float cosine = fabs(its.shFrame.n.dot(lRec.wi));

                // std::cout << "cosine: " << cosine << endl;
    

                // std::cout << "multiplication: " << (lightResult * bsdfResult * cosine).toString() << endl;

                // Color3f mult = lightResult * bsdfResult;
                // std::cout << "lightResult: " << lightResult.toString() << " bsdfResult: " << bsdfResult.toString() <<  " mult: " << mult.toString() << endl;

                L += (lightResult * bsdfResult * cosine);
            }
        }


        return L;
    }

    std::string toString() const {
        return tfm::format("DirectLightEms[]");
    }
};

NORI_REGISTER_CLASS(DirectLightEms, "direct_ems");
NORI_NAMESPACE_END