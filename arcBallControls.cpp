#include "arcBallControls.h"

#include <limits.h>
#include <assert.h>

using namespace ArcBall;

namespace {
    static constexpr float practicallyZero = std::numeric_limits<float>::epsilon() * 10.0f;
}

void ArcBallControls::mapScreenPosToArcBallPosNDC( linAlg::vec3_t& mCurrMouseNDC, const linAlg::vec2_t& relative_screenPos ) {
    // map cursor pos to NDC and project to unit sphere for z coordinate
    mCurrMouseNDC[0] = (2.0f * relative_screenPos[0]) - 1.0f;
    mCurrMouseNDC[1] = 2.0f - (2.0f * relative_screenPos[1]) - 1.0f;

    const float currX2 = mCurrMouseNDC[0] * mCurrMouseNDC[0];
    const float currY2 = mCurrMouseNDC[1] * mCurrMouseNDC[1];

    // http://courses.cms.caltech.edu/cs171/assignments/hw3/hw3-notes/notes-hw3.html
    //mCurrMouseNDC[2] = (currX2 + currY2 <= 1.0f) ? sqrtf( 1.0f - currX2 - currY2 ) : 0.0f;
    // https://www.xarg.org/2021/07/trackball-rotation-using-quaternions/
    mCurrMouseNDC[2] = (currX2 + currY2 <= 0.5f) ? sqrtf( 1.0f - currX2 - currY2 ) : 0.5f / (sqrtf( currX2 + currY2 ));

    linAlg::normalize( mCurrMouseNDC );
}


ArcBallControls::ArcBallControls()
    : mDeadZone( practicallyZero * 1000.0f )
    , mIsActive( true ) {

    resetTrafos();

    setInteractionMode( InteractionModeDesc{ .fullCircle = true, .smooth = true } );

    setRotDampingFactor( 0.875f );
    setMouseSensitivity( 0.866f );
    setMaxTraditionalRotDeg( 360.0f ); 

    mLMBdown = false;

    mRelativeCurrMouseX = 0.0f;
    mRelativeCurrMouseY = 0.0f;
    mPrevRelativeMouseX = mRelativeCurrMouseX;
    mPrevRelativeMouseY = mRelativeCurrMouseY;
    
    mTargetRelativeMouse_dx = 0.0f;
    mTargetRelativeMouse_dy = 0.0f;

    mStartMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 0.0f };
    mCurrMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 0.0f };
}


void ArcBallControls::setRotationPivotWS( const linAlg::vec3_t& pivotWSIn ) { 

#if 0 // STABLE!!!
    mRotationPivotPosArcSpaceWS = pivotWSIn;
#else // UNSTABLE!!!
    if (linAlg::dist( pivotWSIn, mRotationPivotPosArcSpaceWS ) <= std::numeric_limits<float>::epsilon() * 100.0f) { return; }
    linAlg::vec3_t pivotWS = pivotWSIn;
    linAlg::applyTransformationToPoint( getArcRotMat(), &pivotWS, 1 );
    mRotationPivotPosArcSpaceWS = pivotWS;
#endif
}
void ArcBall::ArcBallControls::setRotationPivotArcSpaceWS( const linAlg::vec3_t& pivotArcSpaceWS ) {
    mRotationPivotPosArcSpaceWS = pivotArcSpaceWS;
}


linAlg::vec3_t ArcBall::ArcBallControls::getRotationPivotOffsetArcSpaceWS() { 
    return mRotationPivotPosArcSpaceWS; 
}
linAlg::vec3_t ArcBall::ArcBallControls::getRotationPivotOffsetWS() { 
    linAlg::mat4_t arcRotMat4;
    linAlg::castMatrix( arcRotMat4, getArcRotMat() );
    linAlg::mat4_t invArcRotMat4;
    linAlg::inverse( invArcRotMat4, arcRotMat4 );
    
    linAlg::vec4_t pivotWS{ mRotationPivotPosArcSpaceWS[0], mRotationPivotPosArcSpaceWS[1], mRotationPivotPosArcSpaceWS[2], 1.0f };

    linAlg::applyTransformationToPoint( invArcRotMat4, &pivotWS, 1 );
    return linAlg::vec3_t{ pivotWS[0], pivotWS[1], pivotWS[2] };
}

void ArcBallControls::seamlessSetRotationPivotWS( const linAlg::vec3_t& pivotWSIn, const float& camTiltRadAngle, const float& camDist ) {
    setRotationPivotWS( pivotWSIn );
    commonSeamlessSetRotationPivotWS( camTiltRadAngle, camDist );
}

void ArcBallControls::seamlessSetRotationPivotArcSpaceWS( const linAlg::vec3_t& pivotArcSpaceWS, const float& camTiltRadAngle, const float& camDist ) {
    setRotationPivotArcSpaceWS( pivotArcSpaceWS );
    commonSeamlessSetRotationPivotWS( camTiltRadAngle, camDist );
}

void ArcBall::ArcBallControls::commonSeamlessSetRotationPivotWS( const float& camTiltRadAngle, const float& camDist )
{
    linAlg::vec3_t prevRefPtES{ 0.0f, 0.0f, 0.0f };
    auto viewWithoutArcMat = getViewTranslationMat() * getTiltRotMat();

    linAlg::applyTransformationToPoint( viewWithoutArcMat, &prevRefPtES, 1 );

    calcViewWithoutArcMatFrameMatrices( camTiltRadAngle, { 0.0f, 0.0f, 0.0f }, camDist );

    linAlg::vec3_t newRefPtES{ 0.0f, 0.0f, 0.0f };
    //auto 
    viewWithoutArcMat = getViewTranslationMat() * getTiltRotMat();
    linAlg::applyTransformationToPoint( viewWithoutArcMat, &newRefPtES, 1 );

    auto panDeltaPivotCompensation = prevRefPtES - newRefPtES;
    addPanDelta( panDeltaPivotCompensation );
}

eRetVal ArcBallControls::update( const float deltaTimeSec, 
                                 const float relativeMouseX, 
                                 const float relativeMouseY, 
                                 const float camDist,
                                 const linAlg::vec3_t& camPanDelta,
                                 const float camTiltRadAngle,
                                 const bool LMBpressed, 
                                 const bool RMBpressed ) {

    (void)deltaTimeSec;

    
    mRelativeCurrMouseX = relativeMouseX;
    mRelativeCurrMouseY = relativeMouseY;
    
    const float relative_mouse_dx = (mIsActive) ? (mRelativeCurrMouseX - mPrevRelativeMouseX) : 0.0f;
    const float relative_mouse_dy = (mIsActive) ? (mRelativeCurrMouseY - mPrevRelativeMouseY) : 0.0f;

    if (!mIsActive) {
        mTargetRelativeMouse_dx = 0.0f;
        mTargetRelativeMouse_dy = 0.0f;
    }

    calcArcMat( camTiltRadAngle, relative_mouse_dx, relative_mouse_dy, LMBpressed );

    calcViewWithoutArcMatFrameMatrices( camTiltRadAngle, camPanDelta, camDist );

    mViewRotMat = mViewRotMat * mArcRotMat;
    mViewMat = mViewMat * mArcRotMat;
    

    ////////////////////////////
    // book keeping & updates //
    ////////////////////////////

    mPrevRelativeMouseX = mRelativeCurrMouseX;
    mPrevRelativeMouseY = mRelativeCurrMouseY;

    if (mInteractionModeDesc.smooth) {
        if (fabsf( mTargetRelativeMouse_dx ) > mDeadZone) { // prevent mTargetmouse_dx from becomming too small "#DEN => denormalized" - may have caused the weird disappearance glitch on mouse interaction
            mTargetRelativeMouse_dx *= getRotDampingFactor();
        }
        else {
            mTargetRelativeMouse_dx = 0.0f;
        }
        if (fabsf( mTargetRelativeMouse_dy ) > mDeadZone) { // prevent mTargetmouse_dx from becomming too small "#DEN => denormalized" - may have caused the weird disappearance glitch on mouse interaction
            mTargetRelativeMouse_dy *= getRotDampingFactor();
        }
        else {
            mTargetRelativeMouse_dy = 0.0f;
        }
    } /*else {
        mTargetRelativeMouse_dx = 0.0f;
        mTargetRelativeMouse_dy = 0.0f;
    }*/

    return eRetVal::OK;
}

void ArcBall::ArcBallControls::calcViewWithoutArcMatFrameMatrices( const float camTiltRadAngle, const linAlg::vec3_t& camPanDelta, const float camDist )
{
    linAlg::loadRotationZMatrix( mTiltRotMat, camTiltRadAngle );

#if 1 // works (up to small jumps when resetting pivot anker pos); uses transform from last frame (seems to be okay as well)
    linAlg::mat3x4_t pivotTranslationMatrix;
    auto pivotTranslationPos = linAlg::vec3_t{ -mRotationPivotPosArcSpaceWS[0], -mRotationPivotPosArcSpaceWS[1], -mRotationPivotPosArcSpaceWS[2] };
    linAlg::loadTranslationMatrix( pivotTranslationMatrix, pivotTranslationPos );
    linAlg::mat3x4_t invPivotTranslationMatrix;
    auto invPivotTranslationPos = linAlg::vec3_t{ -pivotTranslationPos[0], -pivotTranslationPos[1], -pivotTranslationPos[2] };
    linAlg::loadTranslationMatrix( invPivotTranslationMatrix, invPivotTranslationPos );
    mTiltRotMat = invPivotTranslationMatrix * mTiltRotMat * pivotTranslationMatrix;
#endif
    //mViewRotMat = mTiltRotMat * mArcRotMat;
    mViewRotMat = mTiltRotMat;

    if (mInteractionModeDesc.smooth) {
        mPanVector[0] += camPanDelta[0];
        mPanVector[1] += camPanDelta[1];
    }
    else {
        mPanVector[0] += camPanDelta[0] / (mPanDampingFactor);
        mPanVector[1] += camPanDelta[1] / (mPanDampingFactor);
    }
    linAlg::vec3_t panVec3{ mPanVector[0], mPanVector[1], camDist + mPanVector[2] };

    linAlg::loadTranslationMatrix( mViewTranslationMat, panVec3 );

    mViewMat = mViewTranslationMat * mViewRotMat;
}

void ArcBall::ArcBallControls::calcArcMat( const float camTiltRadAngle, const float mouse_dx, const float mouse_dy, const bool LMBpressed )
{
    linAlg::mat3_t rolledRefFrameMatT;
    {
        linAlg::mat3x4_t camRollMat;
        linAlg::loadRotationZMatrix( camRollMat, camTiltRadAngle );

        linAlg::mat3_t rolledRefFrameMat;
        linAlg::vec3_t refX;
        linAlg::castVector( refX, camRollMat[0] );
        linAlg::vec3_t refY;
        linAlg::castVector( refY, camRollMat[1] );
        linAlg::vec3_t refZ;
        linAlg::castVector( refZ, camRollMat[2] );
        rolledRefFrameMat[0] = refX;
        rolledRefFrameMat[1] = refY;
        rolledRefFrameMat[2] = refZ;

        linAlg::transpose( rolledRefFrameMatT, rolledRefFrameMat );

        setRefFrameMat( rolledRefFrameMatT );
    }

    if (mInteractionModeDesc.smooth) {
        if (mLMBdown) {
            mTargetRelativeMouse_dx += mouse_dx * mMouseSensitivity; // * deltaTimeSec;
            mTargetRelativeMouse_dy += mouse_dy * mMouseSensitivity; // * deltaTimeSec;
        }
    }
    else {
        mTargetRelativeMouse_dx = mouse_dx * mMouseSensitivity;
        mTargetRelativeMouse_dy = mouse_dy * mMouseSensitivity;
    }

    if (mInteractionModeDesc.fullCircle == true) { // continuous ArcBall rotation with grabbed mouse

        if ((mInteractionModeDesc.smooth && sqrtf( mTargetRelativeMouse_dx * mTargetRelativeMouse_dx + mTargetRelativeMouse_dy * mTargetRelativeMouse_dy ) > mDeadZone) || mLMBdown) {
            //printf( "LMB is down\n" );

            // always reset start to center of ArcBall
            mStartMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 1.0f };

            // only take mouse delta and add it to center of ArcBall
            ArcBallControls::mapScreenPosToArcBallPosNDC( mCurrMouseNDC, linAlg::vec2_t{ 0.5f + mTargetRelativeMouse_dx, 0.5f + mTargetRelativeMouse_dy } );

            linAlg::normalize( mCurrMouseNDC );
            linAlg::normalize( mStartMouseNDC );

            float cosAngle = linAlg::dot( mStartMouseNDC, mCurrMouseNDC );
            assert( cosAngle > 0.0 );
            if (cosAngle < 1.0f - std::numeric_limits<float>::epsilon()) {
                const float cosMousePtDirs = linAlg::minimum( 1.0f, cosAngle ); // <= 1.0 so that arccos doesn't freak out
                const float radMousePtDir = acosf( cosMousePtDirs );
                if (fabsf( radMousePtDir > mDeadZone ))
                {
                    linAlg::vec3_t normMousePtDirs;
                    linAlg::cross( normMousePtDirs, mStartMouseNDC, mCurrMouseNDC );
                    linAlg::normalize( normMousePtDirs );

                    linAlg::mat3x4_t rotArcBallDeltaMat;

                    // bring rotation vector into ref frame
                    linAlg::applyTransformationToPoint( mRefFrameMat, &normMousePtDirs, 1 );
                    linAlg::normalize( normMousePtDirs );

                    linAlg::loadRotationAroundAxis( rotArcBallDeltaMat, normMousePtDirs, radMousePtDir );

                #if 1
                    linAlg::mat3x4_t pivotTranslationMatrix;
                    linAlg::loadTranslationMatrix( pivotTranslationMatrix, linAlg::vec3_t{ -mRotationPivotPosArcSpaceWS[0], -mRotationPivotPosArcSpaceWS[1], -mRotationPivotPosArcSpaceWS[2] } );
                    linAlg::mat3x4_t invPivotTranslationMatrix;
                    linAlg::loadTranslationMatrix( invPivotTranslationMatrix, mRotationPivotPosArcSpaceWS );
                    rotArcBallDeltaMat = invPivotTranslationMatrix * rotArcBallDeltaMat * pivotTranslationMatrix;
                #endif

                    linAlg::mat3x4_t tmpLastRotMat;
                    linAlg::multMatrix( tmpLastRotMat, rotArcBallDeltaMat, mArcRotMat );
                    mArcRotMat = tmpLastRotMat;
                }
            }
        }

        if (!mLMBdown && LMBpressed) {
            //printf( "LMB pressed\n" );
            mLMBdown = true;
        }
        if (mLMBdown && !LMBpressed) {
            //printf( "LMB released\n" );
            mLMBdown = false;
        }
    }
    else { // Traditional Arcball - works, but doesn't spin more than 180° in any dir
        if (mLMBdown) {
            //printf( "LMB is down\n" );
            ArcBallControls::mapScreenPosToArcBallPosNDC( mCurrMouseNDC, linAlg::vec2_t{ mRelativeCurrMouseX, mRelativeCurrMouseY } );

            linAlg::normalize( mCurrMouseNDC );
            linAlg::normalize( mStartMouseNDC );

            linAlg::applyTransformationToPoint( mRefFrameMat, &mCurrMouseNDC, 1 );

            linAlg::normalize( mCurrMouseNDC );


            float cosAngle = linAlg::dot( mStartMouseNDC, mCurrMouseNDC );
            if (cosAngle < 1.0f - std::numeric_limits<float>::epsilon() * 100.0f) {
                const float cosMousePtDirs = linAlg::minimum( 0.9999999f, cosAngle ); // <= 1.0 so that arccos doesn't freak out
                const float radMousePtDir = acosf( cosMousePtDirs ) * (mMaxTraditionalRotDeg * (1.0f / 180.0f));
                linAlg::vec3_t normMousePtDirs;
                linAlg::cross( normMousePtDirs, mStartMouseNDC, mCurrMouseNDC );
                linAlg::normalize( normMousePtDirs );

                // bring rotation vector into ref frame
                linAlg::loadRotationAroundAxis( mCurrRotMat, normMousePtDirs, radMousePtDir );

            #if 1
                linAlg::mat3x4_t pivotTranslationMatrix;
                linAlg::loadTranslationMatrix( pivotTranslationMatrix, linAlg::vec3_t{ -mRotationPivotPosArcSpaceWS[0], -mRotationPivotPosArcSpaceWS[1], -mRotationPivotPosArcSpaceWS[2] } );
                linAlg::mat3x4_t invPivotTranslationMatrix;
                linAlg::loadTranslationMatrix( invPivotTranslationMatrix, mRotationPivotPosArcSpaceWS );
                mCurrRotMat = invPivotTranslationMatrix * mCurrRotMat * pivotTranslationMatrix;
            #endif
            }
        }

        if (!mLMBdown && LMBpressed) {
            //printf( "LMB pressed\n" );
            ArcBallControls::mapScreenPosToArcBallPosNDC( mStartMouseNDC, linAlg::vec2_t{ mRelativeCurrMouseX, mRelativeCurrMouseY } );

            linAlg::applyTransformationToPoint( mRefFrameMat, &mStartMouseNDC, 1 );
            linAlg::normalize( mStartMouseNDC );

            mLMBdown = true;
        }
        if (mLMBdown && !LMBpressed) {
            //printf( "LMB released\n" );
            linAlg::mat3x4_t tmpLastRotMat;
            linAlg::multMatrix( tmpLastRotMat, mCurrRotMat, mPrevRotMat );
            mPrevRotMat = tmpLastRotMat;
            linAlg::loadIdentityMatrix( mCurrRotMat );
            mLMBdown = false;
        }

        linAlg::multMatrix( mArcRotMat, mCurrRotMat, mPrevRotMat );
    }
}

void ArcBall::ArcBallControls::setViewMatrix( const linAlg::mat3x4_t& viewMatrix ) {

    resetTrafos();

    mViewMat = viewMatrix;

    // extract only rotational part
    linAlg::mat3x4_t rotOnlyMat = viewMatrix;
    rotOnlyMat[0][3] = 0.0f;
    rotOnlyMat[1][3] = 0.0f;
    rotOnlyMat[2][3] = 0.0f;
    
    linAlg::mat3_t rotOnlyMat3;
    linAlg::castMatrix( rotOnlyMat3, rotOnlyMat );
    linAlg::mat3_t invRotOnlyMat3;
    linAlg::transpose(invRotOnlyMat3, rotOnlyMat3);

    mCurrRotMat = rotOnlyMat;

    mPanVector = { viewMatrix[0][3], viewMatrix[1][3], viewMatrix[2][3] };

}

void ArcBallControls::setRefFrameMat( const linAlg::mat3_t& refFrameMat ) {
    mRefFrameMat = refFrameMat;
    linAlg::orthogonalize( mRefFrameMat );
}

void ArcBallControls::resetTrafos() {
    
    linAlg::loadIdentityMatrix( mArcRotMat );
    linAlg::loadIdentityMatrix( mTiltRotMat );

    linAlg::loadIdentityMatrix( mViewRotMat );
    linAlg::loadIdentityMatrix( mViewTranslationMat );
    linAlg::loadIdentityMatrix( mViewMat );

    mPanVector = { 0.0f, 0.0f, 0.0f };
    mRotationPivotPosArcSpaceWS = { 0.0f, 0.0f, 0.0f };

    linAlg::loadIdentityMatrix( mCurrRotMat );
    linAlg::loadIdentityMatrix( mPrevRotMat );
    linAlg::loadIdentityMatrix( mRefFrameMat );

    mStartMouseNDC = linAlg::vec3_t{ 0.0f, 0.0f, 1.0f };
    mCurrMouseNDC  = linAlg::vec3_t{ 0.0f, 0.0f, 1.0f };

    mRelativeCurrMouseX = 0;
    mRelativeCurrMouseY = 0;
    mPrevRelativeMouseX = 0;
    mPrevRelativeMouseY = 0;
    mTargetRelativeMouse_dx = 0;
    mTargetRelativeMouse_dy = 0;
}
