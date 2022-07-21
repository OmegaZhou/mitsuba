#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/texture.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/core/warp.h>
#include <random>
MTS_NAMESPACE_BEGIN

class GaussianFunction 
{
public:
	static Float getValue(Float x, Float sigma)
	{
		return math::fastexp(-x * x * 0.5f / (sigma * sigma)) / (math::safe_sqrt((float)(2.0 * M_PI)) * sigma);
	}
	GaussianFunction(Float v = 1)
	{
		sigma = v;
	}
	Float sigma;
	Float getValue(Float x)
	{
		return math::fastexp(-x * x * 0.5f / (sigma * sigma)) / (math::safe_sqrt((float)(2.0 * M_PI)) * sigma);
	}
	void check() 
	{
		ref<Random> random = new Random();
		const int count = 1e7;
		Float sum = 0;
		for (int i = 0; i < count; ++i) {
			Float a = random->nextFloat();
			Float pdf = 0;
			if (a < 0.8) {
				Float x = (random->nextFloat() - 0.5) * 1e2 / sigma;
				pdf = 0.8 * 1e-2 * sigma + 0.2 * 1e-6;
				sum += getValue(x) / pdf;
			}
			else {
				Float x = (random->nextFloat() - 0.5) * 1e6;
				pdf = 0.2 * 1e-6;
				if (x > -50 / sigma && x < 50 / sigma) {
					pdf += 0.8 * 1e-2 * sigma;
				}
				sum += getValue(x) / pdf;
			}
		}
		sum /= count;
		Log(ELogLevel::EDebug, "Sigma %lf Result %lf",sigma, sum);
		
	}
	MTS_DECLARE_CLASS()
};
MTS_IMPLEMENT_CLASS(GaussianFunction, false, GaussianFunction)
class MarschnerBSDF : public BSDF 
{
public:
	MarschnerBSDF(const Properties& props) : BSDF(props)
	{
		alpha[0] = props.getFloat("alpha", -5) * M_PI / 180;
		alpha[1] = -alpha[0] / 2;
		alpha[2] = -3 * alpha[0] * 2;
		beta[0] = props.getFloat("beta", 5) * M_PI / 180;
		beta[1] = beta[0] / 2;
		beta[2] = beta[0] * 2;
		//beta[1] = 6 * M_PI / 180;
		//beta[2] = 15 * M_PI / 180;

		props.getSpectrum("sigmaA", Spectrum(0.0f)).toLinearRGB(sigma_a.x, sigma_a.y, sigma_a.z);
		deltaHm = props.getFloat("deltaHm", 0.5f);
		wc = props.getFloat("wc", 10) * M_PI / 180;
		kg = props.getFloat("Kg", 0.5f);
		deltaEta = props.getFloat("deltaEta", 0.2f);
		eta = props.getFloat("eta", 1.7f);
		radius = props.getFloat("radius", 0.05f);
		Log(ELogLevel::EDebug, "%lf %lf %lf", sigma_a.x, sigma_a.y, sigma_a.z);
	}
	MarschnerBSDF(Stream* stream, InstanceManager* manager): BSDF(stream, manager) 
	{
		configure();
	}
	void configure()
	{
		m_components.push_back(EDiffuseReflection | EFrontSide);
		BSDF::configure();
	}
	virtual Spectrum eval(const BSDFSamplingRecord& bRec, EMeasure measure = ESolidAngle) const 
	{
		auto& n = bRec.its.geoFrame.n;
		auto wo_geo = normalize(bRec.its.geoFrame.toLocal(normalize(bRec.its.toWorld(bRec.wo))));
		auto sin_theta = wo_geo.x;
		auto cos_theta = math::safe_sqrt(1 - sin_theta * sin_theta);
		auto ret = bsdf(bRec) * std::abs(cos_theta);
		//Log(ELogLevel::EDebug, "Bsdf %lf, %lf %lf", ret[0], ret[1], ret[2]);
		return ret;
	}
	virtual Spectrum sample(BSDFSamplingRecord& bRec, const Point2& sample) const 
	{
		Float pdf;
		return this->sample(bRec, pdf, sample);
	}
	virtual Spectrum sample(BSDFSamplingRecord& bRec, Float& pdf, const Point2& sample) const
	{
		//std::default_random_engine e;
		//std::uniform_real_distribution<float> a(0, 1);
		//int i = std::min(2.0f, std::floorf(3.0f * a(e)));
		//std::normal_distribution<float> n(alpha[i], beta[i]);
		//auto wi_geo = normalize(bRec.its.geoFrame.toLocal(normalize(bRec.its.toWorld(bRec.wi))));
		//float theta_h = n(e);
		//float phi = 2 * M_PI * a(e);
		//float theta_r = math::safe_asin(wi_geo.x);
		//float theta = 2 * theta_h - theta_r;
		//float sin_theta = sinf(theta);
		//float cos_theta = cosf(theta);
		//float sin_phi = sinf(phi);
		//float cos_phi = cosf(phi);
		//bRec.wo.x = sin_theta;
		//bRec.wo.y = cos_theta * sin_phi;
		//bRec.wo.z = cos_theta * cos_phi;
		////bRec.wo = warp::squareToUniformSphere(sample);
		//bRec.wo = bRec.its.toLocal(bRec.its.geoFrame.toWorld(bRec.wo));
		bRec.wo = warp::squareToUniformHemisphere(sample);
		bRec.eta = 1.0f;
		bRec.sampledComponent = 0;
		bRec.sampledType = EDiffuseReflection;
		pdf = this->pdf(bRec);
		return bsdf(bRec);
	}
	virtual Float pdf(const BSDFSamplingRecord& bRec, EMeasure measure = ESolidAngle) const
	{
		//auto wi_geo = normalize(bRec.its.geoFrame.toLocal(normalize(bRec.its.toWorld(bRec.wi))));
		//auto wo_geo = normalize(bRec.its.geoFrame.toLocal(normalize(bRec.its.toWorld(bRec.wo))));
		//auto sin_thetai = wi_geo.x;
		//auto sin_thetao = wo_geo.x;
		//auto thetai = math::safe_asin(sin_thetai);
		//auto thetao = math::safe_asin(sin_thetao);
		//auto theta_h = (thetao + thetai) * 0.5f;
		//theta_h = convertAngle(theta_h);
		//Float pdf = 0;
		//for (int i = 0; i < 3; ++i) {
		//	pdf += GaussianFunction::getValue(theta_h - alpha[i], beta[i]);
		//}
		//pdf *=  INV_PI / 6;
		//auto pdf = warp::squareToUniformSpherePdf();
		auto pdf = warp::squareToUniformHemispherePdf();
		return pdf;
	}
	MTS_DECLARE_CLASS()
private:
	// longitudinal width(stdev.)
	Float beta[3];
	//longitudinal shift
	Float alpha[3];
	GaussianFunction g[3];
	Float eta;
	Vector3 sigma_a;
	Float deltaHm;
	Float wc;
	Float kg;
	Float deltaEta;
	Float radius;
	Float convertAngle(Float angle) const
	{
		while (angle < -M_PI) {
			angle += 2.0f * M_PI;
		}
		while (angle > M_PI) {
			angle -= 2.0f * M_PI;
		}
		return angle;
	}
	Spectrum bsdf(const BSDFSamplingRecord& bRec) const
	{
		auto wi_geo = normalize(bRec.its.geoFrame.toLocal(normalize(bRec.its.toWorld(bRec.wi))));
		auto wo_geo = normalize(bRec.its.geoFrame.toLocal(normalize(bRec.its.toWorld(bRec.wo))));
		auto sin_thetai = wi_geo.x;
		auto sin_thetao = wo_geo.x;

		auto thetai = math::safe_asin(sin_thetai);
		auto thetao = math::safe_asin(sin_thetao);
		auto theta_d = (thetao - thetai) * 0.5f;
		auto theta_h = (thetao + thetai) * 0.5f;

		auto phi_vi = normalize(wi_geo - sin_thetai * Vector3(1, 0, 0));
		auto phi_vo = normalize(wo_geo - sin_thetao * Vector3(1, 0, 0));
		auto phi_i = math::safe_acos(phi_vi.z);
		auto phi_o = math::safe_acos(phi_vi.z);
		auto phi = phi_o - phi_i;
		auto phi_h = (phi_i + phi_o) * 0.5f;
		Float sin_theta_d = std::sin(theta_d);
		Float cos_theta_d2 = 1 - sin_theta_d * sin_theta_d;
		Spectrum ret(0.0f);
		phi = convertAngle(phi);
		theta_d = convertAngle(theta_d);
		theta_h = convertAngle(theta_h);
		for (int i = 0; i < 3; ++i) {
			ret += M(i, theta_h) * Np(i, phi, sin_theta_d);
			//ret = Np(1, phi, sin_theta_d);
		}
		//ret /= cos_theta_d2;
		
		return 2.0f * radius * ret;
	}
	Float M(int p, Float theta_h) const
	{
		return GaussianFunction::getValue(theta_h - alpha[p], beta[p]);
	}

	Spectrum Np(int p, Float phi, Float sin_theta) const
	{
		Spectrum ret(0.0f);

		auto cos_theta = math::safe_sqrt(1 - sin_theta * sin_theta);
		Float eta1 = this->eta1(sin_theta, cos_theta);
		Float eta2 = this->eta2(eta1);
		auto hs = calculateH(phi, p, eta1);
		std::vector<Spectrum> cache;
		for (auto& h : hs) {
			auto ap = Ap(p, h, sin_theta, cos_theta, eta1, eta2);
			ret += ap * 0.5f * invdPhidHAbs(h, p, eta1);
			if (p == 2) {
				cache.push_back(ap);
			}
		}

		// Ntrt
		if (p == 2) {

			for (int i = 0; i < 3; ++i) {
				ret[i] = std::isinf(ret[i]) ? 0 : ret[i];
			}
			Float hc, phic, deltaH, t;
			if (eta1 < 2) {
				getPhicAndHc(phic, hc, eta1);
				deltaH = std::min(deltaHm, calcuteDeltaH(wc, hc, eta1));
				t = 1;
			}
			else {
				phic = 0;
				deltaH = deltaHm;
				t = 1 - smoothStep(2, 2 + deltaEta, eta1);
			}
			phic = convertAngle(phic);
			auto g0 = GaussianFunction::getValue(0, wc);
			auto g1 = GaussianFunction::getValue(phi - phic, wc);
			auto g2 = GaussianFunction::getValue(phi + phic, wc);
			ret *= (1 - t * g1 / g0);
			ret *= (1 - t * g2 / g0);
			
			Float scaler = t * kg * deltaH * (g1 + g2);
			
			for (auto& ap : cache) {
				ret += ap * scaler;
			}

		}
		return ret;
	}

	Spectrum Ap(int p, Float h, Float sin_theta, Float cos_theta, Float eta1, Float eta2) const
	{

		if (p == 0) {
			
			return fresnel(eta1, eta2, h) * Spectrum(1.0f);
		}
		else {
			Float inv_eta1 = 1 / eta1;
			Float inv_eta2 = 1 / eta2;
			Float inner_f = fresnel(inv_eta1, inv_eta2, h * inv_eta1);
			Float item1 = (1 - fresnel(eta1, eta2, h)) * (1 - inner_f);
			Float item2 = 1;
			for (int i = 0; i < p - 1; ++i) {
				item2 *= inner_f;
			}
			auto t = T(sigma_a / calCosDelta(eta1, sin_theta, cos_theta), h * inv_eta1);
			auto item3 = Spectrum(1.0f);
			for (int i = 0; i < p; ++i) {
				item3 *= t;
			}
			//Log(ELogLevel::EDebug, "Fresnel %lf %lf %lf %lf %lf", item1, item1 * item2, item1 * item2 * item3[0], item1 * item2 * item3[1], item1 * item2 * item3[2]);
			return item1 * item2 * item3;
		}
		
	}
	Float calCosDelta(Float eta1, Float sin_gamma, Float cos_gamma) const
	{
		return eta1 * cos_gamma / eta;
	}
	Spectrum T(const Vector3& sigma_a, Float h) const
	{
		Spectrum ret;
		Float cos_gamma = math::safe_sqrt(1 - h * h);
		for (int i = 0; i < 3; ++i) {
			ret[i] = math::fastexp(-2 * sigma_a[i] * cos_gamma);
		}
		return ret;
	}

	std::vector<Float> calculateH(Float phi, int p, Float eta) const
	{
		std::vector<Float> result;
		Float gamma;

		if (p == 0) {
			gamma = -phi * 0.5f;
			result.push_back(sinf(gamma));
		}
		else {
			const Float INV_PI3 = INV_PI * INV_PI * INV_PI;
			Float c = math::safe_asin(1 / eta);
			Float a = -8 * p * c * INV_PI3;
			Float b = 6 * p * c * INV_PI - 2;
			Float k = p * M_PI - phi;
			auto tmp = caculateCubicEquation(a, b, k);
			for (auto& i : tmp) {
				Float maxv, minv;
				if (p == 1) {
					maxv = M_PI - 2 * math::safe_asin(1 / eta);
					minv = -maxv;
					maxv += M_PI;
					minv += M_PI;
				}
				else if (p == 2) {
					maxv = INFINITY;
					minv = -INFINITY;
				}
				if (i >= minv && i <= maxv) {
					result.push_back(sinf(i));
				}
			}
		}
		return result;
	}
	
	// at^3+bt+c=0
	std::vector<Float> caculateCubicEquation(Float a, Float b, Float c) const
	{
		Float p = b / a;
		Float q = c / a;
		Float D = 4 * p * p * p + 27 * q * q;
		std::vector<Float> ret;
		if (D > 0) {
			Float a = -q / 2;
			Float b = math::safe_sqrt(q * q / 4 + p * p * p / 27);
			ret.push_back(std::cbrtf(a + b) + std::cbrtf(a - b));
		}
		else if (D == 0) {
			auto a = std::cbrtf(-q / 2.0f);
			ret.push_back(2 * a);
			ret.push_back(-a);
		}
		else {
			Float a = 3 * q / (2 * p) * std::sqrt(-3 / p);
			Float theta = math::safe_acos(a) / 3;
			auto k = 2 * math::safe_sqrt(-p / 3);
			for (int i = 0; i < 3; ++i) {
				ret.push_back(k * std::cos(theta - 2 * M_PI * i / 3));
			}
		}
		return ret;
	} 
	void checkCubicEquation(Float a, Float b, Float c) const
	{
		auto ret = caculateCubicEquation(a, b, c);
		for (auto i : ret) {
			Log(ELogLevel::EDebug,"Result %lf", a * i * i * i + b * i + c);
		}
	}

	Float invdPhidHAbs(Float h, int p, Float eta) const
	{
		if (p == 0) {
			return 0.5f*math::safe_sqrt(1 - h * h);
		}
		else {
			return 0.5f / std::abs((p / math::safe_sqrt(eta * eta - h * h) - 1.0f / math::safe_sqrt(1 - h * h)));
		}
	}

	// gamma = asin(dot(wi, s))
	Float eta1(Float sin_gamma, Float cos_gamma) const
	{
		Float cosv = cos_gamma;
		Float sinv2 = sin_gamma * sin_gamma;
		return math::safe_sqrt(eta * eta - sinv2) / cosv;
	}
	Float eta2(Float eta1) const
	{
		return eta * eta / eta1;
	}
	Float fresnel(Float eta1, Float eta2, Float h) const
	{
		Float sin_gamma_i = h;
		Float sin_gamma_o = h / eta1;
		Float cos_gamma_i = math::safe_sqrt(1 - sin_gamma_i * sin_gamma_i);
		Float cos_gamma_o = math::safe_sqrt(1 - sin_gamma_o * sin_gamma_o);
		Float Rs = (cos_gamma_i - cos_gamma_o * eta1) / (cos_gamma_i + cos_gamma_o * eta1);
		Float Rp = (eta2* cos_gamma_i - cos_gamma_o) / (eta2* cos_gamma_i + cos_gamma_o);
		return 0.5f * (Rs*Rs + Rp*Rp);
	}
	void getPhicAndHc(Float& phi_c, Float& h_c, Float eta1) const
	{
		h_c = math::safe_sqrt((4 - eta1 * eta1) / 3);
		phi_c = 4 * math::safe_asin(h_c / eta1) - 2 * math::safe_asin(h_c) + 2 * M_PI;
	}
	Float calcuteDeltaH(Float wc, Float h_c, Float eta1) const
	{
		auto h2 = h_c * h_c;
		auto d2phidh2 = 4 * h_c / std::powf(eta1 * eta1 - h2, 1.5f) - 2 * h_c / std::powf(1 - h2, 1.5f);
		return 2.0f * math::safe_sqrt(2 * wc / std::abs(d2phidh2));
	}
	Float smoothStep(Float start, Float end, Float value) const
	{
		return math::clamp((value - start) / (end - start), 0.0f, 1.0f);
	}
};

MTS_IMPLEMENT_CLASS_S(MarschnerBSDF, false, BSDF)
MTS_EXPORT_PLUGIN(MarschnerBSDF, "Hair BSDF")
MTS_NAMESPACE_END