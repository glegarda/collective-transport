#include <cmath>
#include <random>
#include "utils.h"

void sleep(sctime& start, float duration)
{
	while (true)
	{
		sctime end = sclock::now();
		std::chrono::duration<float> diff = end - start;
		if (diff.count() >= duration)
		{
			start = sclock::now();
			break;
		}
	}
}

double delta(sctime t1, sctime t2)
{
    return std::chrono::duration<double>(t2 - t1).count();
}

VectorPolar::VectorPolar() : r(0.0f), a(0.0f) {}

VectorPolar::VectorPolar(float radius, float azimuth) : r(radius), a(azimuth)
{
	toMpiPi();
}

VectorPolar::VectorPolar(const VectorPolar& v) : r(v.r), a(v.a)
{
	toMpiPi();
}

VectorPolar::VectorPolar(const b2Vec2& v) :
	r(v.Length()),
	a(std::atan2(v.y, v.x)) {}

VectorPolar::~VectorPolar() {}

void VectorPolar::rotate(float angle)
{
	a += angle;
	toMpiPi();
}

b2Vec2 VectorPolar::to_b2Vec2()
{
	return b2Vec2(r * std::cos(a), r * std::sin(a));
}

VectorPolar VectorPolar::operator+(const VectorPolar& v) const
{
	float x = r * std::cos(a) + v.r * std::cos(v.a);
	float y = r * std::sin(a) + v.r * std::sin(v.a);
	VectorPolar tmp;
	tmp.r = std::sqrt(x * x + y *y);
	tmp.a = std::atan2(y, x);
	return tmp;
}

VectorPolar VectorPolar::operator+=(const VectorPolar &v)
{
	float x = r * std::cos(a) + v.r * std::cos(v.a);
	float y = r * std::sin(a) + v.r * std::sin(v.a);
	this->r = std::sqrt(x * x + y * y);
	this->a = std::atan2(y, x);
	return *this;
}

VectorPolar VectorPolar::operator-(const VectorPolar& v) const
{
	float x = r * std::cos(a) - v.r * std::cos(v.a);
	float y = r * std::sin(a) - v.r * std::sin(v.a);
	VectorPolar tmp;
	tmp.r = std::sqrt(x * x + y *y);
	tmp.a = std::atan2(y, x);
	return tmp;
}

VectorPolar VectorPolar::operator-=(const VectorPolar &v)
{
	float x = r * std::cos(a) - v.r * std::cos(v.a);
	float y = r * std::sin(a) - v.r * std::sin(v.a);
	this->r = std::sqrt(x * x + y * y);
	this->a = std::atan2(y, x);
	return *this;
}

VectorPolar VectorPolar::operator*(const float& f) const
{
	if (f < 0.0f)
	{
		return VectorPolar(-f * r, a + M_PI);
	}

	return VectorPolar(f * r, a);
}

std::ostream& operator<<(std::ostream& os, const VectorPolar& v)
{
	os << "[" << v.r << "," << v.a << "]";
	return os;
}

void VectorPolar::toMpiPi()
{
	while (a >= M_PI)
	{
		a -= (2.0f * M_PI);
	}

	while (a < -M_PI)
	{
		a += (2.0f * M_PI);
	}
}

// ** Should try to put these somewhere safer... ** //
std::random_device rd;
std::mt19937 gen(rd());

// Random float generator in the range [fmin, fmax)
float rndf(float fmin, float fmax)
{
    std::uniform_real_distribution<float> dis(fmin, fmax);
    return dis(gen);
}

// Random int generator in the range [imin, imax]
int rndi(int imin, int imax)
{
    std::uniform_int_distribution<int> dis(imin, imax);
    return dis(gen);
}

// Random int generator in the range [imin, imax]
float rndif(int imin, int imax)
{
    std::uniform_int_distribution<int> dis(imin, imax);
    return static_cast<float>(dis(gen));
}
void seed(int seed)
{
    gen.seed(seed);
}

void process_args(int argc, char **argv, CommandLineParams &p)
{
    for(int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-h"))
        {
            printf("Usage: exe [options]\n"
                   "    -h          Print this message\n"
                   "    -g          Use gui (default true for sim, false for gp)\n"
                   "    -ng         Don't use gui\n"
                   "    -px         Use new new prox method\n"
                   "    -b          Run benchmark\n"
                   "    -tf <file>  Use file for behaviour tree\n"
                   "    -t <num>    Simulation time (seconds, default 120)\n"
                   "    -r <num>    Random number generator seed (default random)\n"
                   "    -l <file>   Log file\n"
                   "    -ll <num>   Log level 0 - (default) positions only\n"
                   "    -ph <hz,vi,pi>  Physics engine\n"
                   "                hz - integrator freq, must be multiple of 10 (default 30)\n"
                   "                vi - vel iter (default 8)\n"
                   "                pi - pos iter (default 3)\n"
                   "    -v          verbose\n"
                   "    -rp         random placement\n"
                   "    -n <num>    number of agents (only with -rp, default 16)\n"
                   );
            exit(0);
            
        }
        else if (!strcmp(argv[i], "-g"))
            p.gui = true;
        else if (!strcmp(argv[i], "-ng"))
            p.gui = false;
        else if (!strcmp(argv[i], "-px"))
            p.new_prox = true;
        else if (!strcmp(argv[i], "-b"))
            p.benchmark = true;
        else if (!strcmp(argv[i], "-tf"))
        {
            if (++i >= argc)
            {
                printf("Missing treefile!\n");
                exit(1);
            }
            p.treefile = std::string(argv[i]);
        }
        else if (!strcmp(argv[i], "-t"))
        {
            if (++i >= argc)
            {
                printf("Missing simulation time!\n");
                exit(1);
            }
            p.simtime = atof(argv[i]);
        }
        else if (!strcmp(argv[i], "-r"))
        {
            if (++i >= argc)
            {
                printf("Missing Random seed!\n");
                exit(1);
            }
            p.randseed = atoi(argv[i]);
            seed(p.randseed);
        }
        else if (!strcmp(argv[i], "-l"))
        {
            if (++i >= argc)
            {
                printf("Missing log file!\n");
                exit(1);
            }
            p.logfile = std::string(argv[i]);
        }
        else if (!strcmp(argv[i], "-ll"))
        {
            if (++i >= argc)
            {
                printf("Missing log level!\n");
                exit(1);
            }
            p.loglevel = atoi(argv[i]);
        }
        else if (!strcmp(argv[i], "-ph"))
        {
            if (++i >= argc)
            {
                printf("Missing physics engine tuple!\n");
                exit(1);
            }
            std::string s(argv[i]);
            int pos = 0, commas = 0;
            while ((pos = s.find(",", pos + 1)) != std::string::npos) commas++;
            if (commas != 2)
            {
                printf("Incorrect number of physics parameters; expecting hz,vi,pi!\n");
                exit(1);
            }
            p.hz = std::stol(s.substr(0, pos = s.find(","))); s.erase(0, pos + 1);
            p.vi = std::stol(s.substr(0, pos = s.find(","))); s.erase(0, pos + 1);
            p.pi = std::stol(s.substr(0, pos = s.find(",")));
            printf("Physics hz:%d vi:%d pi:%d\n", p.hz, p.vi, p.pi);
        }
        else if (!strcmp(argv[i], "-v"))
            p.verbose = true;
        else if (!strcmp(argv[i], "-rp"))
            p.rand_position = true;
        else if (!strcmp(argv[i], "-n"))
        {
            if (++i >= argc)
            {
                printf("Missing number of agents!\n");
                exit(1);
            }
            p.num_agents = atoi(argv[i]);
        }
    }
}
