#ifndef ALGORITHM_STATS_H
#define ALGORITHM_STATS_H

#include <cmath>
#include <chrono>

namespace OsrPlanner {

    class FunctionCallStats{
        public:
            FunctionCallStats() {
                numCalls = 0;      
                totalCallTime = 0;
                minCallTime = 1000000;
                maxCallTime = 0;                
            };

            double getAvgCallTime() {
                if (numCalls < 1)
                    return 0;
                
                return (double) totalCallTime / (double) numCalls;
            };

            int getMinCallTime() {
                if (numCalls < 1)
                    return 0;
                
                return minCallTime;
            }

            void updateCallTime(std::chrono::time_point<std::chrono::high_resolution_clock> start) {
                auto end = getTime();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();

                numCalls++;
                totalCallTime += duration;

                int intDuration = (int) duration;
                
                if (intDuration < minCallTime)
                    minCallTime = duration;
                
                if (intDuration > maxCallTime)
                    maxCallTime = duration;
            };

            std::chrono::time_point<std::chrono::high_resolution_clock> getTime() {
                return std::chrono::high_resolution_clock::now();
            };

            int numCalls;
            int totalCallTime;
            int minCallTime;
            int maxCallTime;
    };

    class AlgorithmStats{
        public:
            AlgorithmStats(){
                iterations = 0;                
                rsShots = 0;
                rsShotsSuccessful = 0;
            };        

            int iterations;
            int rsShots;
            int rsShotsSuccessful;
            FunctionCallStats updateH;            
            FunctionCallStats viz;
    };

}


#endif //ALGORITHM_STATS_H