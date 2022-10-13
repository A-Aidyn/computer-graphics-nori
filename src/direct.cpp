#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLight : public Integrator {
public:
    DirectLight(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f L(0.0);
        Intersection its;
        if(!scene->rayIntersect(ray, its))
            return L;
        
        if(!(its.mesh) || !(its.mesh -> getBSDF()))
            return L;

        BSDF *bsdf = (BSDF*)(its.mesh -> getBSDF());

        // std::cout << (scene -> getLights()).size() << endl;

        for(auto light : scene -> getLights()) {
            EmitterQueryRecord lRec;
            lRec.ref = its.p;
            Point2f sample;
            Color3f lightResult = light -> sample(lRec, sample);

            // std::cout << "lightResult: " << lightResult.toString() << endl;
            Ray3f fromPointToLight;
            fromPointToLight.o = its.p;
            fromPointToLight.d = lRec.wi;
            fromPointToLight.maxt = sqrt((lRec.p - lRec.ref).dot((lRec.p - lRec.ref)));
            fromPointToLight.update();

            Intersection newIts;

            if (scene->rayIntersect(fromPointToLight, newIts))
                continue;

            Vector3f wo = (ray.o - lRec.ref).normalized();
            // Vector3f wo = (ray.o - lRec.ref);
            BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(wo), ESolidAngle);
            // Point2f uv = sphericalCoordinates(lRec.wi);
            // bRec.uv = uv;
            // bRec.p = its.p;
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

        // std::cout << "L: " << L.toString() << endl;

        return L;
    }

    std::string toString() const {
        return tfm::format("DirectLight[]");
    }
};

NORI_REGISTER_CLASS(DirectLight, "direct");
NORI_NAMESPACE_END