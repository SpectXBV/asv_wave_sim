#ifndef WAVE_PROBE_HH_
#define WAVE_PROBE_HH   

#include<gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
    class WaveProbePrivate;

    class WaveProbe:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
    {
        // Constructor
        public: WaveProbe();

        
    }
}
}
}

}