#ifndef CONSTANTS_H
#define CONSTANTS_H
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   \todo All constants need to be checked and documented
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid

#include <cmath>

namespace OsrPlanner {
    namespace Constants {
        static const int headings = 72;

        static const float tieBreaker = 0.01;

        /// [c*M_PI] --- The discretization value of heading (goal condition)
        static const float deltaHeadingRad = 2 * M_PI / (float)headings;
        /// [c*M_PI] --- The heading part of the goal condition
        static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
        /// [°] --- The discretization value of the heading (goal condition)
        static const float deltaHeadingDeg = 360 / (float)headings;

        /// [#] --- The sqrt of the number of discrete positions per cell
        static const int positionResolution = 10;
        /// [#] --- The number of discrete positions per cell
        static const int positions = positionResolution * positionResolution;

        /// A structure describing the relative position of the occupied cell based on the center of the vehicle
        struct relPos {
            /// the x position relative to the center
            int x;
            /// the y position relative to the center
            int y;
        };

        /// A structure capturing the lookup for each theta configuration
        struct config {
            /// the number of cells occupied by this configuration of the vehicle
            int length;
            /*!
                \var relPos pos[64]
                \brief The maximum number of occupied cells
                \todo needs to be dynamic
            */
            relPos pos[64];
        };

        // ____________________________________________
        // COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
        /// A structure to express colors in RGB values
        struct color {
        /// the red portion of the color
        float red;
        /// the green portion of the color
        float green;
        /// the blue portion of the color
        float blue;
        };
        /// A definition for a color used for visualization
        static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
        /// A definition for a color used for visualization
        static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
        /// A definition for a color used for visualization
        static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
        /// A definition for a color used for visualization
        static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
        /// A definition for a color used for visualization
        static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};


    }
}

#endif // CONSTANTS_H