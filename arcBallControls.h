#ifndef _ARCBALLCONTROLS_H_9ec4f00a_2117_4578_937e_9f4fb94dc759
#define _ARCBALLCONTROLS_H_9ec4f00a_2117_4578_937e_9f4fb94dc759


#include "eRetVal_ArcBall.h"
#include "../math/linAlg.h" // TODO: can we make it work like this: "#include <linAlg.h>"

#include <stdint.h>

namespace ArcBall {
    struct ArcBallControls {

        struct InteractionModeDesc {
            bool fullCircle;  // mouse may go off screen, arc ball will continue to rotate
            bool smooth;// rotation keeps going on when mouse button is no longer pressed
        };
        
        static void mapScreenPosToArcBallPosNDC( linAlg::vec3_t& currMouseNDC, const linAlg::vec2_t& screenPos, const int32_t fbWidth, const int32_t fbHeight );

        ArcBallControls();
        eRetVal update( const float deltaTimeSec, 
                        const float mouseX, 
                        const float mouseY,     
                        const float camDist,
                        const linAlg::vec3_t& camPanDelta,
                        const float camTiltRadAngle,
                        const bool LMBpressed, 
                        const bool RMBpressed, 
                        const int32_t screenW, 
                        const int32_t screenH );
        
        const linAlg::mat3x4_t& getRotationMatrix() const { return mViewRotMat; }
        const linAlg::mat3x4_t& getViewMatrix() const { return mViewMat; }

        void setRefFrameMat( const linAlg::mat3_t& refFrameMat );
        void setRotationPivotOffset( const linAlg::vec3_t& offset ) { mRotationPivotOffset = offset; }

        void setRotDampingFactor( const float dampingFactor ) { mRotDampingFactor = dampingFactor; }
        float getRotDampingFactor() const { return mRotDampingFactor; }

        void setPanDampingFactor( const float dampingFactor ) { mPanDampingFactor = dampingFactor; }
        float getPanDampingFactor() const { return mPanDampingFactor; }

        void setMouseSensitivity( const float mouseSensitivity ) { mMouseSensitivity = mouseSensitivity; }

        void setInteractionMode( const InteractionModeDesc modeDesc ) { mInteractionModeDesc = modeDesc; }
        InteractionModeDesc getInteractionMode() const { return mInteractionModeDesc; }

        void setMaxTraditionalRotDeg( const float maxTraditionalRotDeg ) { mMaxTraditionalRotDeg = maxTraditionalRotDeg;  }

        void setDeadZone( const float deadZone ) { mDeadZone = deadZone; }
        float getDeadZone() const { return mDeadZone; }

        void resetTrafos();
        void setActive( const bool isActive ) { mIsActive = isActive; }

        float getTargetMovement_dx() const { return mTargetMouse_dx; };
        float getTargetMovement_dy() const { return mTargetMouse_dy; };


    private:
        linAlg::mat3x4_t mArcRotMat;
        linAlg::mat3x4_t mTiltRotMat;
        linAlg::mat3x4_t mViewRotMat;
        linAlg::mat3x4_t mViewTranslationMat;
        linAlg::mat3x4_t mViewMat;

        linAlg::vec3_t   mPanVector;
        
        // only for clamp mouse interaction ("traditional" arc ball)    
        linAlg::mat3x4_t mCurrRotMat;
        linAlg::mat3x4_t mPrevRotMat;

        linAlg::mat3_t mRefFrameMat; // for camera rolling - without camera rolling, this may stay a unit matrix
        linAlg::vec3_t mRotationPivotOffset;

        linAlg::vec3_t mStartMouseNDC;
        linAlg::vec3_t mCurrMouseNDC;

        float mCurrMouseX;
        float mCurrMouseY;
        float mPrevMouseX;
        float mPrevMouseY;
        float mTargetMouse_dx;
        float mTargetMouse_dy;
        float mMouseSensitivity;
        float mRotDampingFactor;
        float mPanDampingFactor;
        float mDeadZone;

        float mMaxTraditionalRotDeg; // 180.0f for traditional arcBall, 360.0f for one full rotation per mouse-drag (stronger movement)

        InteractionModeDesc mInteractionModeDesc;
        bool  mLMBdown;
        bool  mIsActive;
    };
}
#endif // _ARCBALLCONTROLS_H_9ec4f00a_2117_4578_937e_9f4fb94dc759
