#include "arcBallControls.h"

#include <assert.h>

using namespace ArcBall;

void ArcBallControls::mapScreenPosToArcBallPosNDC( linAlg::vec3_t& mCurrMouseNDC, const linAlg::vec2_t& screenPos, const int32_t fbWidth, const int32_t fbHeight ) {
    // map cursor pos to NDC and project to unit sphere for z coordinate
    mCurrMouseNDC[0] = (2.0f * screenPos[0] / (static_cast<float>(fbWidth) - 1.0f)) - 1.0f;
    mCurrMouseNDC[1] = 2.0f - (2.0f * screenPos[1] / (static_cast<float>(fbHeight) - 1.0f)) - 1.0f;
    const float currX2 = mCurrMouseNDC[0] * mCurrMouseNDC[0];
    const float currY2 = mCurrMouseNDC[1] * mCurrMouseNDC[1];

    // http://courses.cms.caltech.edu/cs171/assignments/hw3/hw3-notes/notes-hw3.html
    //mCurrMouseNDC[2] = (currX2 + currY2 <= 1.0f) ? sqrtf( 1.0f - currX2 - currY2 ) : 0.0f;
    // https://www.xarg.org/2021/07/trackball-rotation-using-quaternions/
    mCurrMouseNDC[2] = (currX2 + currY2 <= 0.5f) ? sqrtf( 1.0f - currX2 - currY2 ) : 0.5f / (sqrtf( currX2 + currY2 ));

    linAlg::normalize( mCurrMouseNDC );
}


ArcBallControls::ArcBallControls() {

    resetTrafos();

    setInteractionMode( MouseInteractMode::wrapAround );
    //setInteractionMode( MouseInteractMode::clamp );

    setDampingFactor( 0.91f );
    setMouseSensitivity( 0.23f );
    mLMBdown = false;

    mCurrMouseX = 0.0f;
    mCurrMouseY = 0.0f;
    mPrevMouseX = mCurrMouseX;
    mPrevMouseY = mCurrMouseY;
    
    mTargetMouse_dx = 0.0f;
    mTargetMouse_dy = 0.0f;

    mStartMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 0.0f };
    mCurrMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 0.0f };
}

eRetVal ArcBallControls::update( const float mouseX, const float mouseY, const bool LMBpressed, const bool RMBpressed, const int32_t screenW, const int32_t screenH ) {

    mCurrMouseX = mouseX;
    mCurrMouseY = mouseY;
    const float mouse_dx = (mCurrMouseX - mPrevMouseX) * mMouseSensitivity;
    const float mouse_dy = (mCurrMouseY - mPrevMouseY) * mMouseSensitivity;

    if (mLMBdown) {
        mTargetMouse_dx += mouse_dx;
        mTargetMouse_dy += mouse_dy;
    }

    if (mMouseInteractMode == MouseInteractMode::wrapAround) { // continuous ArcBall rotation with grabbed mouse

        //if (mLMBdown) { // when using mouse_dx directly
        if ((mTargetMouse_dx * mTargetMouse_dx + mTargetMouse_dy * mTargetMouse_dy > 0.001) || mLMBdown) {
            //printf( "LMB is down\n" );

            // always reset start to cener of ArcBall
            mStartMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 1.0f };

            // only take mouse delta and add it to center of ArcBall
            ArcBallControls::mapScreenPosToArcBallPosNDC( mCurrMouseNDC, linAlg::vec2_t{ screenW / 2 + mTargetMouse_dx, screenH / 2 + mTargetMouse_dy }, screenW, screenH );

            linAlg::normalize( mCurrMouseNDC );
            linAlg::normalize( mStartMouseNDC );
            //volatile 
            float cosAngle = linAlg::dot( mStartMouseNDC, mCurrMouseNDC );
            assert( cosAngle > 0.0 );
            //if (cosAngle < 1.0f - std::numeric_limits<float>::epsilon() * 100.0f) {
            if (cosAngle <= 1.0f) {
                const float cosMousePtDirs = linAlg::minimum( 1.0f, cosAngle ); // <= 1.0 so that arccos doesn't freak out
                const float radMousePtDir = acosf( cosMousePtDirs );
                if (fabsf( radMousePtDir > 0.01 )) {
                    linAlg::vec3_t normMousePtDirs;
                    linAlg::cross( normMousePtDirs, mStartMouseNDC, mCurrMouseNDC );
                    linAlg::normalize( normMousePtDirs );

                    linAlg::mat3x4_t rotArcBallDeltaMat;

                    // bring rotation vector into ref frame
                    linAlg::applyTransformation( mRefFrameMat, &normMousePtDirs, 1 );
                    linAlg::normalize( normMousePtDirs );

                    linAlg::loadRotationAroundAxis( rotArcBallDeltaMat, normMousePtDirs, radMousePtDir );

                    linAlg::mat3x4_t tmpLastRotMat;
                    linAlg::multMatrix( tmpLastRotMat, rotArcBallDeltaMat, mArcRotMat );
                    mArcRotMat = tmpLastRotMat;

                    //linAlg::orthogonalize( mArcRotMat );
                }
            }
        }

        if (!mLMBdown && LMBpressed) {
            printf( "LMB pressed\n" );
            mLMBdown = true;
        }
        if (mLMBdown && !LMBpressed) {
            printf( "LMB released\n" );
            mLMBdown = false;
        }
    } else { // Traditional Arcball - works, but doesn't spin more than 180� in any dir
        if (mLMBdown) {
            //printf( "LMB is down\n" );
            ArcBallControls::mapScreenPosToArcBallPosNDC( mCurrMouseNDC, linAlg::vec2_t{ mCurrMouseX, mCurrMouseY }, screenW, screenH );

            linAlg::normalize( mCurrMouseNDC );
            linAlg::normalize( mStartMouseNDC );

            linAlg::applyTransformation( mRefFrameMat, &mCurrMouseNDC, 1 );
            linAlg::normalize( mCurrMouseNDC );
            //linAlg::applyTransformation( mRefFrameMat, &mStartMouseNDC, 1 );
            //linAlg::normalize( mStartMouseNDC );


            float cosAngle = linAlg::dot( mStartMouseNDC, mCurrMouseNDC );
            if (cosAngle < 1.0f - std::numeric_limits<float>::epsilon() * 100.0f) {
                const float cosMousePtDirs = linAlg::minimum( 1.0f, cosAngle ); // <= 1.0 so that arccos doesn't freak out
                const float radMousePtDir = acosf( cosMousePtDirs );
                linAlg::vec3_t normMousePtDirs;
                linAlg::cross( normMousePtDirs, mStartMouseNDC, mCurrMouseNDC );
                linAlg::normalize( normMousePtDirs );

                // bring rotation vector into ref frame
                //linAlg::applyTransformation( mRefFrameMat, &normMousePtDirs, 1 );
                //linAlg::normalize( normMousePtDirs );

                linAlg::loadRotationAroundAxis( mCurrRotMat, normMousePtDirs, radMousePtDir );
                //linAlg::orthogonalize( mCurrRotMat );
            }
        }

        if (!mLMBdown && LMBpressed) {
            printf( "LMB pressed\n" );
            ArcBallControls::mapScreenPosToArcBallPosNDC( mStartMouseNDC, linAlg::vec2_t{ mCurrMouseX, mCurrMouseY }, screenW, screenH );

            linAlg::applyTransformation( mRefFrameMat, &mStartMouseNDC, 1 );
            linAlg::normalize( mStartMouseNDC );

            mLMBdown = true;
        }
        if (mLMBdown && !LMBpressed) {
            printf( "LMB released\n" );
            linAlg::mat3x4_t tmpLastRotMat;
            linAlg::multMatrix( tmpLastRotMat, mCurrRotMat, mPrevRotMat );
            mPrevRotMat = tmpLastRotMat;
            linAlg::loadIdentityMatrix( mCurrRotMat );
            mLMBdown = false;
        }

        linAlg::multMatrix( mArcRotMat, mCurrRotMat, mPrevRotMat );
        //linAlg::orthogonalize( mArcRotMat );
    }

    mPrevMouseX = mCurrMouseX;
    mPrevMouseY = mCurrMouseY;

    //if (!mLMBdown) {
    const float practicallyZero = 0.00001f;
    if (fabsf( mTargetMouse_dx ) > practicallyZero) { // prevent mTargetmouse_dx from becomming too small "#DEN => denormalized" - may have caused the weird disappearance glitch on mouse interaction
        mTargetMouse_dx *= 0.91f;
    }
    if (fabsf( mTargetMouse_dy ) > practicallyZero) { // prevent mTargetmouse_dx from becomming too small "#DEN => denormalized" - may have caused the weird disappearance glitch on mouse interaction
        mTargetMouse_dy *= 0.91f;
    }
    //}

    return eRetVal::OK();
}

void ArcBallControls::setRefFrameMat( const linAlg::mat3_t& refFrameMat )
{
    mRefFrameMat = refFrameMat;
    linAlg::orthogonalize( mRefFrameMat );
}

void ArcBallControls::resetTrafos() {
    linAlg::loadIdentityMatrix( mArcRotMat );

    linAlg::loadIdentityMatrix( mCurrRotMat );
    linAlg::loadIdentityMatrix( mPrevRotMat );

    linAlg::loadIdentityMatrix( mRefFrameMat );

    mStartMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 1.0f };
    mCurrMouseNDC  = linAlg::vec3_t{ 0.0f, 0.0f, 1.0f };

    mCurrMouseX = 0;
    mCurrMouseY = 0;
    mPrevMouseX = 0;
    mPrevMouseY = 0;
    mTargetMouse_dx = 0;
    mTargetMouse_dy = 0;

}