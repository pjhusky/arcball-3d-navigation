#ifndef _ARCBALLCONTROLS_H_9ec4f00a_2117_4578_937e_9f4fb94dc759
#define _ARCBALLCONTROLS_H_9ec4f00a_2117_4578_937e_9f4fb94dc759

// Short Explanation about coord spaces to use
//  mViewMat = mViewTranslationMat * mTiltRotMat * mArcRotMat;
//  where we can shorthand mViewRotMat = mTiltRotMat * mArcRotMat;
//
// i refer to the space after arcRot matrix is applied to as "ArcSpaceWS"
// it is like a model matrix still (the very first transformation in the view transform)
//
// setting the pivot from the outside is preferably exposed as expecting a WS position (preferably wrt ease of use and clarity of the interface to the ArcBall)
// however, we actually need the pivot pos to be in ArcSpaceWS
// now, setting the pivot often would mean that the arcRot matrix is applied often and that can cause float-wobble 
// where the object starts oscillating wildly and just goes off to some crazy dist
// for this reason we skip identical re-sets of pivot pos
// when picking a new pivot on a model surface from screen and transforming to "world", instead of taking
// {mViewTranslationMat * mTiltRotMat * mArcRotMat}^(-1)  ... the inverse of the full view matrix transform which really goes to WS
// we can just use 
// {mViewTranslationMat * mTiltRotMat}^(-1) ... to "only" go to ArcSpaceWS => this space we don't need to transform the point that is passed in all the time from WS to ArcSpaceWS
// as we already expect it to be in ArcSpaceWS, whereas in the first version with the clean interface, we get the true WS, and apply the arcRot matrix to get the pos to ArcSpaceWS



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

        // "view-matrix" part
        void calcViewWithoutArcMatFrameMatrices( const float camTiltRadAngle, const linAlg::vec3_t& camPanDelta, const float camDist );

        // "model-matrix" part
        void calcArcMat( const float camTiltRadAngle, const float mouse_dx, const float mouse_dy, const int32_t screenW, const int32_t screenH, const bool LMBpressed );
        
        const linAlg::mat3x4_t& getArcRotMat() const { return mArcRotMat; } // model matrix part - can be thought of as rotated object
        const linAlg::mat3x4_t& getTiltRotMat() const { return mTiltRotMat; }
        const linAlg::mat3x4_t& getViewRotMat() const { return mViewRotMat; }
        const linAlg::mat3x4_t& getViewTranslationMat() const { return mViewTranslationMat; }
        
        const linAlg::mat3x4_t& getViewMatrix() const { return mViewMat; }
        void setViewMatrix( const linAlg::mat3x4_t& viewMatrix );

        void addPanDelta( const linAlg::vec3_t& delta ) { mPanVector = mPanVector + delta; }
        

        void setRefFrameMat( const linAlg::mat3_t& refFrameMat );
        
        void setRotationPivotWS( const linAlg::vec3_t& pivotWSIn );
        void setRotationPivotArcSpaceWS( const linAlg::vec3_t& pivotArcSpaceWS ); // Model-matrix part more stable, but less easy to use from the outside

        linAlg::vec3_t getRotationPivotOffsetArcSpaceWS();
        linAlg::vec3_t getRotationPivotOffsetWS();

        void seamlessSetRotationPivotWS( const linAlg::vec3_t& pivotWS, const float& camTiltRadAngle, const float& camDist );
        void seamlessSetRotationPivotArcSpaceWS( const linAlg::vec3_t& pivotArcSpaceWS, const float& camTiltRadAngle, const float& camDist );
        void commonSeamlessSetRotationPivotWS( const float& camTiltRadAngle, const float& camDist );
        //void seamlessSetRotationPivotWS( const linAlg::vec3_t& pivotWS, const float& camTiltRadAngle, const float& camDist );

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
        linAlg::vec3_t mRotationPivotPosArcSpaceWS;

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
