#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibility : public Integrator {
public:
    AverageVisibility(const PropertyList &props) {
        rayLength = props.getFloat("length");
        std::cout << "Parameter value was : " << rayLength << std::endl;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(1.0f);

        /* Return the component-wise absolute
           value of the shading normal as a color */
        Normal3f n = its.shFrame.n;
        
        Ray3f newRay;
        newRay.o = its.p;
        newRay.d = Warp::sampleUniformHemisphere(sampler, n);
        newRay.maxt = rayLength;
        newRay.update();

        Intersection newIts;

        if (!scene->rayIntersect(newRay, newIts))
            return Color3f(1.0f);
        else
            return Color3f(0.0f);
    }

    std::string toString() const {
        return tfm::format(
            "AverageVisibilityIntegrator[\n"
            "  rayLength = \"%s\"\n"
            "]",
            rayLength
        );
    }
protected:
    float rayLength;
};

NORI_REGISTER_CLASS(AverageVisibility, "av");
NORI_NAMESPACE_END