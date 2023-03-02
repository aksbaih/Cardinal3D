
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    const Vec3 reflected{-dir.x, clamp(dir.y, 0.f, std::numeric_limits<float>::max()), -dir.z};
    return reflected.unit();
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    if(out_dir.y < 0.f) index_of_refraction = 1.f / index_of_refraction;
    const float incidentSin = Vec3{out_dir.x, 0.f, out_dir.z}.norm();
    const float refractedSin = incidentSin / index_of_refraction;
    Vec3 refractedDir{-out_dir.x, 0.f, -out_dir.z};
    refractedDir *= 1.f / incidentSin * refractedSin;
    refractedDir.y = -1.f * sign(out_dir.y) * sqrtf(1.f - refractedDir.norm_squared());
    return refractedDir;
}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.attenuation = evaluate(out_dir, ret.direction);
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
    ret.direction = reflect(out_dir); // What direction should we sample incoming light from?
    const float cosIn = ret.direction.y;
    ret.attenuation = reflectance / cosIn; // What is the ratio of reflected/incoming light?
    ret.pdf = 1.f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6

    // Implement glass BSDF.
    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    bool _;

    const float etaI = out_dir.y > 0.f ? 1.f : index_of_refraction;
    const float etaT = out_dir.y > 0.f ? index_of_refraction : 1.f;
    const Vec3 refracted = refract(out_dir, index_of_refraction, _);
    const float cosI = abs(out_dir.y);
    const float cosT = abs(refracted.y);

    const float rPara = (etaT * cosI - etaI * cosT) / (etaT * cosI + etaI * cosT);
    const float rPerp = (etaI * cosI - etaT * cosT) / (etaT * cosI + etaI * cosT);
    const float Fr = .5f * (rPara * rPara + rPerp * rPerp);

    bool chooseReflected = RNG::coin_flip(Fr);

    BSDF_Sample ret;
    ret.direction = chooseReflected ? reflect(out_dir) : refracted;
    ret.attenuation =
        chooseReflected ? reflectance / ret.direction.y : transmittance / ret.direction.y;
    ret.pdf = chooseReflected ? Fr : (1.f - Fr);
    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    BSDF_Sample ret;
    ret.attenuation = Spectrum(); // What is the ratio of reflected/incoming light?
    ret.direction = Vec3();       // What direction should we sample incoming light from?
    ret.pdf = 0.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
