/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>
#include <mitsuba/render/renderproc.h>
MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("Path tracer", "Average path length", EAverage);
struct PathItem
{
    PathItem() : m_nextQ(1.0f), m_throughput(0.0f), m_driectLightValue(0.0f), m_lightBsdfValue(0.0f), m_lightWeight(0.0f), m_hitLightValue(0.0f), m_hitLightWeight(0.0f), m_bsdfValue(0.0f),
        m_lightWo(0.0f), m_bsdfWo(0.0f), m_otherLightValue(0.f), m_bsdfPdf(0.0f), m_useLightBsdfPdf(false), m_useHitLightPdf(false), m_lightUpdateFlag(false), rRecType(0), m_intersection(), m_ray()
    {

    }
    Spectrum m_throughput;

    Spectrum m_driectLightValue;
    Float m_lightWeight;
    Spectrum m_lightBsdfValue;

    Spectrum m_hitLightValue;
    Float m_hitLightWeight;
    Spectrum m_bsdfValue;
    Float m_bsdfPdf;

    Vector3 m_lightWo;
    Vector3 m_bsdfWo;

    Spectrum m_otherLightValue;

    Intersection m_intersection;
    RayDifferential m_ray;
    float m_eta;
    Float m_nextQ;
    int rRecType;
    bool m_scattered;
    bool m_useLightBsdfPdf;
    bool m_useHitLightPdf;
    bool m_lightUpdateFlag;
    
};
struct PathCache
{
    bool m_bsdfUpdate;
    bool m_lightUpdate;
    bool m_isInit;

    
    PathCache() :m_bsdfUpdate(true), m_lightUpdate(true), m_isInit(false), m_endLoc(0) {

    }
    PathItem& back()
    {
        if (m_endLoc==0) {
            SLog(ELogLevel::EError, "Out of range");
        }
        return m_items[m_endLoc-1];
    }
    PathItem& operator[](size_t i) {
        if (i >= m_endLoc) {
            SLog(ELogLevel::EError, "Out of range");
        }
        return m_items[i];
    }
    const PathItem& operator[](size_t i)const {
        if (i >= m_endLoc) {
            SLog(ELogLevel::EError, "Out of range");
        }
        return m_items[i];
    }
    int size()const {
        return m_endLoc;
    }
    void setEnding(int endLoc)
    {
        m_endLoc = endLoc;
    }
    void addItem()
    {
        if (m_endLoc == m_items.size()) {
            m_items.push_back(PathItem());
        }
        else {
            m_items[m_endLoc] = PathItem();
        }
        ++m_endLoc;
    }
    void clear()
    {
        m_endLoc = 0;
    }
    void init()
    {
        //m_items.reserve(8);
    }
private:
    int m_endLoc;
    std::vector<PathItem> m_items;
};
class PathCacheManager
{
public:
    static PathCacheManager& getInstance() {
        static PathCacheManager manager;
        return manager;
    }
    static void updateStatus(unsigned int width, unsigned int height, unsigned int sampleNum,bool bsdfUpdate, bool lightUpdate) {
        auto& manager = getInstance();
        if ((!manager.m_isInit)||(width!=manager.m_width||height!=manager.m_height||sampleNum!=manager.m_sampleNum)) {
            manager.m_cache.resize(width);
            for (int i = 0; i < width; ++i) {
                manager.m_cache[i].resize(height);
                for (int j = 0; j < height; ++j) {
                    manager.m_cache[i][j].resize(sampleNum);
                }
            }
            manager.m_width = width;
            manager.m_height = height;
            manager.m_sampleNum = sampleNum;
            manager.m_isInit = false;
        }
        for (auto& items : manager.m_cache) {
            for (auto& item : items) {
                for (auto& cache : item) {
                    cache.m_bsdfUpdate = cache.m_bsdfUpdate ? cache.m_bsdfUpdate : bsdfUpdate;
                    cache.m_lightUpdate = cache.m_lightUpdate ? cache.m_lightUpdate : lightUpdate;
                    if (!manager.m_isInit) {
                        cache.m_isInit = false;
                    }
                }
            }
        }
        manager.m_isInit = true;
    }
    bool isInit() const{
        return m_isInit;
    }
    PathCache& getCache(unsigned int x, unsigned int y, unsigned int sampleNum) {
        return m_cache[x][y][sampleNum];
    }
    std::vector<PathCache>& getSamplesCache(unsigned int x, unsigned int y) {
        return m_cache[x][y];
    }
    void calMemory() {
        long long memory = 0;
        for (auto& vec : m_cache) {
            for (auto& v : vec) {
                for (auto& item : v) {
                    memory += item.size() * sizeof(PathItem);
                }
            }
        }
        SLog(ELogLevel::EDebug, "Memory cost:%lf MB", memory / 1024.0 / 1024.0);
    }
private:
    PathCacheManager(): m_isInit(false), m_width(0),m_height(0),m_sampleNum(0) {

    }
    bool m_isInit;
    int m_width, m_height, m_sampleNum;
    std::vector<std::vector<std::vector<PathCache>>> m_cache;
};
/*! \plugin{path}{Path tracer}
 * \order{2}
 * \parameters{
 *     \parameter{maxDepth}{\Integer}{Specifies the longest path depth
 *         in the generated output image (where \code{-1} corresponds to $\infty$).
 *         A value of \code{1} will only render directly visible light sources.
 *         \code{2} will lead to single-bounce (direct-only) illumination,
 *         and so on. \default{\code{-1}}
 *     }
 *     \parameter{rrDepth}{\Integer}{Specifies the minimum path depth, after
 *        which the implementation will start to use the ``russian roulette''
 *        path termination criterion. \default{\code{5}}
 *     }
 *     \parameter{strictNormals}{\Boolean}{Be strict about potential
 *        inconsistencies involving shading normals? See the description below
 *        for details.\default{no, i.e. \code{false}}
 *     }
 *     \parameter{hideEmitters}{\Boolean}{Hide directly visible emitters?
 *        See page~\pageref{sec:hideemitters} for details.
 *        \default{no, i.e. \code{false}}
 *     }
 * }
 *
 * This integrator implements a basic path tracer and is a \emph{good default choice}
 * when there is no strong reason to prefer another method.
 *
 * To use the path tracer appropriately, it is instructive to know roughly how
 * it works: its main operation is to trace many light paths using \emph{random walks}
 * starting from the sensor. A single random walk is shown below, which entails
 * casting a ray associated with a pixel in the output image and searching for
 * the first visible intersection. A new direction is then chosen at the intersection,
 * and the ray-casting step repeats over and over again (until one of several
 * stopping criteria applies).
 * \begin{center}
 * \includegraphics[width=.7\textwidth]{images/integrator_path_figure.pdf}
 * \end{center}
 * At every intersection, the path tracer tries to create a connection to
 * the light source in an attempt to find a \emph{complete} path along which
 * light can flow from the emitter to the sensor. This of course only works
 * when there is no occluding object between the intersection and the emitter.
 *
 * This directly translates into a category of scenes where
 * a path tracer can be expected to produce reasonable results: this is the case
 * when the emitters are easily ``accessible'' by the contents of the scene. For instance,
 * an interior scene that is lit by an area light will be considerably harder
 * to render when this area light is inside a glass enclosure (which
 * effectively counts as an occluder).
 *
 * Like the \pluginref{direct} plugin, the path tracer internally relies on multiple importance
 * sampling to combine BSDF and emitter samples. The main difference in comparison
 * to the former plugin is that it considers light paths of arbitrary length to compute
 * both direct and indirect illumination.
 *
 * For good results, combine the path tracer with one of the
 * low-discrepancy sample generators (i.e. \pluginref{ldsampler},
 * \pluginref{halton}, or \pluginref{sobol}).
 *
 * \paragraph{Strict normals:}\label{sec:strictnormals}
 * Triangle meshes often rely on interpolated shading normals
 * to suppress the inherently faceted appearance of the underlying geometry. These
 * ``fake'' normals are not without problems, however. They can lead to paradoxical
 * situations where a light ray impinges on an object from a direction that is classified as ``outside''
 * according to the shading normal, and ``inside'' according to the true geometric normal.
 *
 * The \code{strictNormals}
 * parameter specifies the intended behavior when such cases arise. The default (\code{false}, i.e. ``carry on'')
 * gives precedence to information given by the shading normal and considers such light paths to be valid.
 * This can theoretically cause light ``leaks'' through boundaries, but it is not much of a problem in practice.
 *
 * When set to \code{true}, the path tracer detects inconsistencies and ignores these paths. When objects
 * are poorly tesselated, this latter option may cause them to lose a significant amount of the incident
 * radiation (or, in other words, they will look dark).
 *
 * The bidirectional integrators in Mitsuba (\pluginref{bdpt}, \pluginref{pssmlt}, \pluginref{mlt} ...)
 * implicitly have \code{strictNormals} set to \code{true}. Hence, another use of this parameter
 * is to match renderings created by these methods.
 *
 * \remarks{
 *    \item This integrator does not handle participating media
 *    \item This integrator has poor convergence properties when rendering
 *    caustics and similar effects. In this case, \pluginref{bdpt} or
 *    one of the photon mappers may be preferable.
 * }
 */
class CachedMIPathTracer : public MonteCarloIntegrator {
public:
    CachedMIPathTracer(const Properties &props)
        : MonteCarloIntegrator(props) {
        m_width = props.getInteger("width", 768);
        m_height = props.getInteger("height", 576);
        m_sampleNum = props.getInteger("sampleNum", 16);
        auto bsdfUpdate = props.getBoolean("bsdfUpdate", true);
        auto lightUpdate = props.getBoolean("lightUpdate", true);
        if (!PathCacheManager::getInstance().isInit()) {
            bsdfUpdate = true;
            lightUpdate = true;
        }
        PathCacheManager::updateStatus(m_width, m_height, m_sampleNum, bsdfUpdate, lightUpdate);
        Log(ELogLevel::EDebug, "Update bsdf: %d  light: %d  rrdepth %d", (int)bsdfUpdate, (int)lightUpdate, m_rrDepth);
    }

    /// Unserialize from a binary data stream
    CachedMIPathTracer(Stream *stream, InstanceManager *manager)
        : MonteCarloIntegrator(stream, manager) { }
    Spectrum Li(const RayDifferential& r, RadianceQueryRecord& rRec) const{
        return Spectrum(0.0f);
    }
    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec, PathCache& cache) const {

        Spectrum Li(0.0f);
        if (!cache.m_isInit) {
            Li = LiInit(r, rRec, cache);
        }
        else {
            if (cache.m_bsdfUpdate && cache.m_lightUpdate) {
                Li = LiWithTotalUpdate(r, rRec, cache);
            }
            else if (cache.m_bsdfUpdate && (!cache.m_lightUpdate)) {
                Li = LiWithUpdateBsdf(r, rRec, cache);
            }
            else if ((!cache.m_bsdfUpdate) && (!cache.m_lightUpdate)) {
                Li = LiWithoutUpdate(r, rRec, cache);
            }
            else if ((!cache.m_bsdfUpdate) && (cache.m_lightUpdate)) {
                Li = LiWithUpdateLight(r, rRec, cache);
            }
        }

        cache.m_bsdfUpdate = false;
        cache.m_lightUpdate = false;
        cache.m_isInit = true;
        /* Store statistics */
        avgPathLength.incrementBase();
        avgPathLength += rRec.depth;

        return Li;
    }

    inline Float miWeight(Float pdfA, Float pdfB) const {
        pdfA *= pdfA;
        pdfB *= pdfB;
        return pdfA / (pdfA + pdfB);
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        MonteCarloIntegrator::serialize(stream, manager);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "MIPathTracer[" << endl
            << "  maxDepth = " << m_maxDepth << "," << endl
            << "  rrDepth = " << m_rrDepth << "," << endl
            << "  strictNormals = " << m_strictNormals << endl
            << "]";
        return oss.str();
    }
    void renderBlock(const Scene* scene,
        const Sensor* sensor, Sampler* sampler, ImageBlock* block,
        const bool& stop, const std::vector< TPoint2<uint8_t> >& points) const {
        //for (auto emitter : scene->getEmitters()) {
        //    auto val = emitter->evalPosition(PositionSamplingRecord());
        //    Log(ELogLevel::EDebug, "fuck %lf %lf %lf", val[0], val[1], val[2]);
        //}
        //for (auto shape : scene->getShapes()) {
        //    if (shape->isEmitter()) {
        //        auto emitter = shape->getEmitter();
        //        auto val = emitter->evalPosition(PositionSamplingRecord());
        //        Log(ELogLevel::EDebug, "fuck %lf %lf %lf", val[0], val[1], val[2]);
        //    }

        //}
        //Log(ELogLevel::EDebug, "fuck2 %x", scene);
        Float diffScaleFactor = 1.0f /
            std::sqrt((Float)sampler->getSampleCount());

        bool needsApertureSample = sensor->needsApertureSample();
        bool needsTimeSample = sensor->needsTimeSample();

        RadianceQueryRecord rRec(scene, sampler);
        Point2 apertureSample(0.5f);
        Float timeSample = 0.5f;
        RayDifferential sensorRay;

        block->clear();

        uint32_t queryType = RadianceQueryRecord::ESensorRay;

        if (!sensor->getFilm()->hasAlpha()) /* Don't compute an alpha channel if we don't have to */
            queryType &= ~RadianceQueryRecord::EOpacity;
        for (size_t i = 0; i < points.size(); ++i) {
            Point2i offset = Point2i(points[i]) + Vector2i(block->getOffset());
            if (stop)
                break;
            sampler->generate(offset);
            auto& caches = PathCacheManager::getInstance().getSamplesCache(offset.x, offset.y);
            
            for (size_t j = 0; j < sampler->getSampleCount(); j++) {
                rRec.newQuery(queryType, sensor->getMedium());
                Point2 samplePos(Point2(offset) + Vector2(rRec.nextSample2D()));
                //Log(ELogLevel::EDebug, "pos %u %u offset %u %u sample %lf %lf ", points[i].x, points[i].y, offset.x, offset.y, samplePos.x, samplePos.y);
                if (needsApertureSample)
                    apertureSample = rRec.nextSample2D();
                if (needsTimeSample)
                    timeSample = rRec.nextSample1D();

                Spectrum spec = sensor->sampleRayDifferential(
                    sensorRay, samplePos, apertureSample, timeSample);

                sensorRay.scaleDifferential(diffScaleFactor);

                spec *= Li(sensorRay, rRec, caches[j]);
                block->put(samplePos, spec, rRec.alpha);
                sampler->advance();
            }
            
        }
    }
    bool render(Scene* scene,
        RenderQueue* queue, const RenderJob* job,
        int sceneResID, int sensorResID, int samplerResID) {
        ref<Scheduler> sched = Scheduler::getInstance();
        ref<Sensor> sensor = static_cast<Sensor*>(sched->getResource(sensorResID));
        ref<Film> film = sensor->getFilm();

        size_t nCores = sched->getCoreCount();
        const Sampler* sampler = static_cast<const Sampler*>(sched->getResource(samplerResID, 0));
        size_t sampleCount = sampler->getSampleCount();

        Log(EInfo, "Starting render job (%ix%i, " SIZE_T_FMT " %s, " SIZE_T_FMT
            " %s, " SSE_STR ") ..", film->getCropSize().x, film->getCropSize().y,
            sampleCount, sampleCount == 1 ? "sample" : "samples", nCores,
            nCores == 1 ? "core" : "cores");
        /* This is a sampling-based integrator - parallelize */
        ref<ParallelProcess> proc = new BlockedRenderProcess(job,
            queue, scene->getBlockSize());
        int integratorResID = sched->registerResource(this);
        int newSceneID = sched->registerResource(scene);
        proc->bindResource("integrator", integratorResID);
        proc->bindResource("scene", newSceneID);
        proc->bindResource("sensor", sensorResID);
        proc->bindResource("sampler", samplerResID);
        scene->bindUsedResources(proc);
        bindUsedResources(proc);
        sched->schedule(proc);

        m_process = proc;
        sched->wait(proc);
        m_process = NULL;
        sched->unregisterResource(integratorResID);
        sched->unregisterResource(newSceneID);

        PathCacheManager::getInstance().calMemory();
        return proc->getReturnStatus() == ParallelProcess::ESuccess;
    }
    MTS_DECLARE_CLASS()
private:
    int m_width;
    int m_height;
    int m_sampleNum;
    void LiLoop(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache, RayDifferential& ray, bool& scattered, Spectrum& throughput, Spectrum& Li, float& eta) const
    {
        const Scene* scene = rRec.scene;
        Intersection& its = rRec.its;
        bool flag = false;
        while (rRec.depth <= m_maxDepth || m_maxDepth < 0) {

            if (!its.isValid()) {
                /* If no intersection could be found, potentially return
                   radiance from a environment luminaire if it exists */
                if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
                    && (!m_hideEmitters || scattered)) {
                    auto value = scene->evalEnvironment(ray);
                    Li += throughput * value;
                }
                break;
            }
            cache.addItem();
            auto& item = cache.back();
            item.m_intersection = its;
            item.m_ray = ray;
            item.m_throughput = throughput;
            item.m_eta = eta;
            item.m_scattered = scattered;
            item.rRecType = rRec.type;
            item.m_lightUpdateFlag = flag;
            const BSDF* bsdf = its.getBSDF(ray);

            /* Possibly include emitted radiance if requested */
            if (its.isEmitter() && (rRec.type & RadianceQueryRecord::EEmittedRadiance)
                && (!m_hideEmitters || scattered)) {
                auto value = its.Le(-ray.d);
                Li += throughput * value;
                item.m_otherLightValue += value;
            }


            /* Include radiance from a subsurface scattering model if requested */
            if (its.hasSubsurface() && (rRec.type & RadianceQueryRecord::ESubsurfaceRadiance)) {
                auto value = its.LoSub(scene, rRec.sampler, -ray.d, rRec.depth);
                Li += throughput * value;
                item.m_otherLightValue += value;
            }


            if ((rRec.depth >= m_maxDepth && m_maxDepth > 0)
                || (m_strictNormals && dot(ray.d, its.geoFrame.n)
                    * Frame::cosTheta(its.wi) >= 0)) {

                /* Only continue if:
                   1. The current path length is below the specifed maximum
                   2. If 'strictNormals'=true, when the geometric and shading
                      normals classify the incident direction to the same side */
                break;
            }

            /* ==================================================================== */
            /*                     Direct illumination sampling                     */
            /* ==================================================================== */

            /* Estimate the direct illumination if this is requested */
            DirectSamplingRecord dRec(its);

            if (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance &&
                (bsdf->getType() & BSDF::ESmooth)) {
                Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());

                if (!value.isZero()) {
                    const Emitter* emitter = static_cast<const Emitter*>(dRec.object);

                    /* Allocate a record for querying the BSDF */
                    auto wo = its.toLocal(dRec.d);
                    BSDFSamplingRecord bRec(its, wo, ERadiance);

                    /* Evaluate BSDF * cos(theta) */
                    const Spectrum bsdfVal = bsdf->eval(bRec);


                    /* Prevent light leaks due to the use of shading normals */
                    if (!bsdfVal.isZero() && (!m_strictNormals
                        || dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

                        /* Calculate prob. of having generated that direction
                           using BSDF sampling */
                        Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
                            ? bsdf->pdf(bRec) : 0;

                        /* Weight using the power heuristic */
                        Float weight = miWeight(dRec.pdf, bsdfPdf);
                        Li += throughput * value * bsdfVal * weight;
                        item.m_lightBsdfValue = bsdfVal;
                        item.m_lightWeight = weight;
                        item.m_driectLightValue = value;
                        item.m_lightWo = wo;
                    }
                }
            }

            /* ==================================================================== */
            /*                            BSDF sampling                             */
            /* ==================================================================== */

            /* Sample BSDF * cos(theta) */
            Float bsdfPdf;
            BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
            Spectrum bsdfWeight = bsdf->sample(bRec, bsdfPdf, rRec.nextSample2D());
            item.m_bsdfWo = bRec.wo;
            item.m_bsdfValue = bsdfWeight;
            item.m_bsdfPdf = bsdfPdf;

            if (bsdfWeight.isZero())
                break;

            scattered |= bRec.sampledType != BSDF::ENull;

            /* Prevent light leaks due to the use of shading normals */
            const Vector wo = its.toWorld(bRec.wo);
            Float woDotGeoN = dot(its.geoFrame.n, wo);
            if (m_strictNormals && woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
                break;

            bool hitEmitter = false;
            Spectrum value;

            /* Trace a ray in this direction */
            ray = Ray(its.p, wo, ray.time);
            if (scene->rayIntersect(ray, its)) {
                /* Intersected something - check if it was a luminaire */
                if (its.isEmitter()) {
                    value = its.Le(-ray.d);
                    dRec.setQuery(ray, its);
                    hitEmitter = true;
                }
            }
            else {
                /* Intersected nothing -- perhaps there is an environment map? */
                const Emitter* env = scene->getEnvironmentEmitter();
                //item.m_lightUpdateFlag |= true;
                if (env) {
                    if (m_hideEmitters && !scattered)
                        break;

                    value = env->evalEnvironment(ray);
                    if (!env->fillDirectSamplingRecord(dRec, ray))
                        break;
                    hitEmitter = true;
                }
                else {
                    break;
                }
            }
            item.m_lightUpdateFlag |= hitEmitter;
            /* Keep track of the throughput and relative
               refractive index along the path */
            throughput *= bsdfWeight;
            eta *= bRec.eta;

            /* If a luminaire was hit, estimate the local illumination and
               weight using the power heuristic */
            if (hitEmitter &&
                (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance)) {
                /* Compute the prob. of generating that direction using the
                   implemented direct illumination sampling technique */
                const Float lumPdf = (!(bRec.sampledType & BSDF::EDelta)) ?
                    scene->pdfEmitterDirect(dRec) : 0;
                Li += throughput * value * miWeight(bsdfPdf, lumPdf);
                item.m_hitLightValue = value;
                item.m_hitLightWeight = miWeight(bsdfPdf, lumPdf);
            }

            /* ==================================================================== */
            /*                         Indirect illumination                        */
            /* ==================================================================== */

            /* Set the recursive query type. Stop if no surface was hit by the
               BSDF sample or if indirect illumination was not requested */
            if (!its.isValid() || !(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
                break;
            rRec.type = RadianceQueryRecord::ERadianceNoEmission;

            if (rRec.depth++ >= m_rrDepth) {
                /* Russian roulette: try to keep path weights equal to one,
                   while accounting for the solid angle compression at refractive
                   index boundaries. Stop with at least some probability to avoid
                   getting stuck (e.g. due to total internal reflection) */

                Float q = std::min(throughput.max() * eta * eta, (Float)0.95f);
                item.m_nextQ = q;
                if (rRec.nextSample1D() >= q)
                    break;
                throughput /= q;
            }
            flag = false;
        }
    }

    // 假定更新后的光源不会挡住物体
    Spectrum LiWithUpdateLight(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache) const
    {
        Spectrum Li(0.0f);
        int depth;
        auto scene = rRec.scene;
        for (depth = 0; depth < cache.size(); ++depth) {
            auto& item = cache[depth];
            auto& its = item.m_intersection;
            auto& throughput = item.m_throughput;
            auto& scattered = item.m_scattered;
            auto& ray = item.m_ray;
            //当击中光源或没击中物体时
            //由于光源更新，可能此时打中物体
            if (item.m_lightUpdateFlag||(!its.isValid())) {
                break;
            }
            if (throughput.isZero()) {
                depth = cache.size();
                break;
            }
            const BSDF* bsdf = its.getBSDF(ray);
            DirectSamplingRecord dRec(its);

            item.m_lightBsdfValue = Spectrum(0.0f);
            item.m_lightWeight = 0;
            item.m_driectLightValue = Spectrum(0.0f);
            item.m_lightWo = Vector();
            if (item.rRecType & RadianceQueryRecord::EDirectSurfaceRadiance &&
                (bsdf->getType() & BSDF::ESmooth)) {
                Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());

                if (!value.isZero()) {
                    const Emitter* emitter = static_cast<const Emitter*>(dRec.object);
                    /* Allocate a record for querying the BSDF */
                    auto wo = its.toLocal(dRec.d);
                    BSDFSamplingRecord bRec(its, wo, ERadiance);
                    /* Evaluate BSDF * cos(theta) */
                    const Spectrum bsdfVal = bsdf->eval(bRec);
                    /* Prevent light leaks due to the use of shading normals */
                    if (!bsdfVal.isZero() && (!m_strictNormals
                        || dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

                        /* Calculate prob. of having generated that direction
                           using BSDF sampling */
                        Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
                            ? bsdf->pdf(bRec) : 0;

                        /* Weight using the power heuristic */
                        Float weight = miWeight(dRec.pdf, bsdfPdf);
                        Li += throughput * value * bsdfVal * weight;
                        item.m_lightBsdfValue = bsdfVal;
                        item.m_lightWeight = weight;
                        item.m_driectLightValue = value;
                        item.m_lightWo = wo;
                    }
                }
            }
            // 若bsdf采样cache采样到了光源，则cache失效，因而不需要计算bsdf采样到光源的情况
        }
        
        updateLight(r, rRec, cache, depth, Li);

        return Li;
    }
    void updateLight(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache, int depth, Spectrum& Li) const 
    {

        if (depth != cache.size() || cache.size() == 0) {
            Spectrum throughput(1.0f);
            Float eta = 1.0f;
            RayDifferential ray(r);
            bool scattered = false;
            if (depth == 0) {
                rRec.rayIntersect(ray);
                ray.mint = Epsilon;
            }
            else {
                auto& item = cache[depth];
                rRec.depth = depth;
                throughput = item.m_throughput;
                eta = item.m_eta;
                scattered = item.m_scattered;
                ray = item.m_ray;
                rRec.its = item.m_intersection;
                rRec.type = item.rRecType;
                rRec.dist = item.m_intersection.t;
            }
            cache.setEnding(depth);
            LiLoop(r, rRec, cache, ray, scattered, throughput, Li, eta);
        }

    }
    Spectrum LiInit(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache) const
    {
        cache.clear();

        /* Some aliases and local variables */
        const Scene* scene = rRec.scene;
        Intersection& its = rRec.its;

        RayDifferential ray(r);

        
        Spectrum Li(0.0f);
        bool scattered = false;
       
        /* Perform the first ray intersection (or ignore if the
           intersection has already been provided). */
        rRec.rayIntersect(ray);
        
        ray.mint = Epsilon;

        Spectrum throughput(1.0f);
        Float eta = 1.0f;
        LiLoop(r, rRec, cache, ray, scattered, throughput, Li, eta);
        

        /* Store statistics */
        avgPathLength.incrementBase();
        avgPathLength += rRec.depth;

        return Li;
    }
    Spectrum LiWithUpdateBsdf(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache) const
    {
        Spectrum Li(0.0f);
        if (cache.size() == 0) {
            RayDifferential ray(r);
            auto scene = rRec.scene;
            bool scattered = false;
            if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
                && (!m_hideEmitters || scattered)) {
                auto value = scene->evalEnvironment(ray);
                Li += value;
            }
        }
        bool flag = false;
        // 假定bsdf只改变参数的情况下，对bsdf采样的分布函数影响不大，因此可以直接使用原重要性权重
        // 重要性权重只影响蒙特卡洛积分方差，不改变期望
        for (int i = 0; i < cache.size(); ++i) {
            auto& item = cache[i];
            auto& its = item.m_intersection;
            auto& throughput = item.m_throughput;

            RayDifferential& ray = item.m_ray;
            Li += throughput * item.m_otherLightValue;
            if (!its.isValid()) {
                break;
            }
            if (throughput.isZero()) {
                i = cache.size();
                break;
            }
            const BSDF* bsdf = its.getBSDF(ray);

            
            // 采样直接光
            {
                Spectrum value = item.m_driectLightValue;
                /* Allocate a record for querying the BSDF */
                if (!value.isZero()) {
                    BSDFSamplingRecord bRec(its, item.m_lightWo, ERadiance);
                    /* Evaluate BSDF * cos(theta) */
                    const Spectrum bsdfVal = bsdf->eval(bRec);
                    Float weight = item.m_lightWeight;
                    Li += throughput * value * bsdfVal * weight;
                    item.m_lightBsdfValue = bsdfVal;
                }

                /* Prevent light leaks due to the use of shading normals */
            }
            /* ==================================================================== */
            /*                            BSDF sampling                             */
            /* ==================================================================== */

            // 采样时若击中光源
            // bsdfWeight = bsdf*cos/pdf
            // cache.m_hitLightWeights[i] = miWeight(bsdfPdf, lumPdf)
            //if (Li.isNaN()) {
            //    Log(ELogLevel::EDebug, "fuck1");
            //}
            if (!item.m_bsdfWo.isZero()) {
                BSDFSamplingRecord bRec(its, item.m_bsdfWo, ERadiance);
                Spectrum bsdfWeight = bsdf->eval(bRec) / item.m_bsdfPdf;
                Spectrum value = item.m_hitLightValue;
                Li += bsdfWeight * throughput * value * item.m_hitLightWeight;
                if (i + 1 != cache.size()) {
                    cache[i + 1].m_throughput = throughput * bsdfWeight / item.m_nextQ;
                    if (cache[i + 1].m_throughput.isNaN()) {
                        auto s = throughput.toString();
                    }
                }
                item.m_bsdfValue = bsdfWeight;
                if (bsdfWeight.isZero()) {
                    break;
                }
            }
            //if (Li.isNaN()) {
            //    Log(ELogLevel::EDebug, "fuck2");
            //}
            
        }
        return Li;
    }

    Spectrum LiWithTotalUpdate(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache) const 
    {
        Spectrum Li(0.0f);
        // 假定bsdf只改变参数的情况下，对bsdf采样的分布函数影响不大，因此可以直接使用原重要性权重
        // 重要性权重只影响蒙特卡洛积分方差，不改变期望
        int depth;
        auto scene = rRec.scene;
        for (depth = 0; depth < cache.size(); ++depth) {
            auto& item = cache[depth];
            auto& its = item.m_intersection;
            const auto& throughput = item.m_throughput;
            const auto& scattered = item.m_scattered;
            const auto& ray = item.m_ray;
            //当击中光源或没击中物体时
            //由于光源更新，可能此时打中物体
            if (item.m_lightUpdateFlag || (!its.isValid())) {
                break;
            }
            if (throughput.isZero()) {
                depth = cache.size();
                break;
            }
            const BSDF* bsdf = its.getBSDF(ray);
            DirectSamplingRecord dRec(its);

            item.m_lightBsdfValue = Spectrum(0.0f);
            item.m_lightWeight = 0;
            item.m_driectLightValue = Spectrum(0.0f);
            item.m_lightWo = Vector();
            if (item.rRecType & RadianceQueryRecord::EDirectSurfaceRadiance &&
                (bsdf->getType() & BSDF::ESmooth)) {
                Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());

                if (!value.isZero()) {
                    const Emitter* emitter = static_cast<const Emitter*>(dRec.object);
                    /* Allocate a record for querying the BSDF */
                    auto wo = its.toLocal(dRec.d);
                    BSDFSamplingRecord bRec(its, wo, ERadiance);
                    /* Evaluate BSDF * cos(theta) */
                    const Spectrum bsdfVal = bsdf->eval(bRec);
                    /* Prevent light leaks due to the use of shading normals */
                    if (!bsdfVal.isZero() && (!m_strictNormals
                        || dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

                        /* Calculate prob. of having generated that direction
                           using BSDF sampling */
                        Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
                            ? bsdf->pdf(bRec) : 0;

                        /* Weight using the power heuristic */
                        Float weight = miWeight(dRec.pdf, bsdfPdf);
                        Li += throughput * value * bsdfVal * weight;
                        item.m_lightBsdfValue = bsdfVal;
                        item.m_lightWeight = weight;
                        item.m_driectLightValue = value;
                        item.m_lightWo = wo;
                    }
                }

            }

            /* ==================================================================== */
            /*                            BSDF sampling                             */
            /* ==================================================================== */

            // 采样时若击中光源
            // bsdfWeight = bsdf*cos/pdf
            // cache.m_hitLightWeights[i] = miWeight(bsdfPdf, lumPdf)
            //if (Li.isNaN()) {
            //    Log(ELogLevel::EDebug, "fuck1");
            //}
            if (!item.m_bsdfWo.isZero()) {
                BSDFSamplingRecord bRec(its, item.m_bsdfWo, ERadiance);
                Spectrum bsdfWeight = bsdf->eval(bRec) / item.m_bsdfPdf;
                Spectrum value = item.m_hitLightValue;
                Li += bsdfWeight * throughput * value * item.m_hitLightWeight;
                if (depth + 1 != cache.size()) {
                    cache[depth + 1].m_throughput = cache[depth].m_throughput * bsdfWeight / item.m_nextQ;
                }
                item.m_bsdfValue = bsdfWeight;
                if (bsdfWeight.isZero()) {
                    break;
                }
                
            }
        }
        updateLight(r, rRec, cache, depth, Li);
        return Li;
    }
    Spectrum LiWithoutUpdate(const RayDifferential& r, RadianceQueryRecord& rRec, PathCache& cache) const
    {
        Spectrum Li(0.0f);

        if (cache.size() == 0) {
            RayDifferential ray(r);
            auto scene = rRec.scene;
            bool scattered = false;
            if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
                && (!m_hideEmitters || scattered)) {
                auto value = scene->evalEnvironment(ray);
                Li += value;
            }
        }
        for (int i = 0; i < cache.size(); ++i) {
            auto& item = cache[i];
            auto& throughput = item.m_throughput;
            if (throughput.isZero()) {
                i = cache.size();
                break;
            }
            Li += throughput * item.m_otherLightValue;
            if (Li.isNaN()) {
                auto str1 = throughput.toString();
                auto str2 = item.m_otherLightValue.toString();
                Log(ELogLevel::EDebug, "fuck1 %s %s", str1.c_str(), str2.c_str());
                break;
            }
            // 采样直接光
            {
                Spectrum value = item.m_driectLightValue;
                const Spectrum bsdfVal = item.m_lightBsdfValue;
                Float weight = item.m_lightWeight;
                Li += throughput * value * bsdfVal * weight;
                /* Prevent light leaks due to the use of shading normals */
            }
            if (Li.isNaN()) {
                Log(ELogLevel::EDebug, "fuck2");
                break;
            }
            /* ==================================================================== */
            /*                            BSDF sampling                             */
            /* ==================================================================== */

            Spectrum bsdfWeight = item.m_bsdfValue;
            Spectrum value = item.m_hitLightValue;
            Li += bsdfWeight * throughput * value * item.m_hitLightWeight;
            if (Li.isNaN()) {
                Log(ELogLevel::EDebug, "fuck3");
                break;
            }
        }
        return Li;
    }
};

MTS_IMPLEMENT_CLASS_S(CachedMIPathTracer, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(CachedMIPathTracer, "Cached MI path tracer");
MTS_NAMESPACE_END
