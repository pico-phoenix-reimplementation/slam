/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#ifdef USE_GRAPHICS
#include <pangolin/pangolin.h>
#endif
#include <mutex>
#include <vector>

namespace ORB_SLAM3
{

class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

    long unsigned int mnId;

    SET_MAP_POINT mspMapPoints;
    SET_KEY_FRAME mspKeyFrames;

    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


   #ifdef USE_GRAPHICS
    GLubyte* mThumbnail;

#endif

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;

};

// pico4-linux: ORB-SLAM3 never frees what it culls. Every keyframe or map point
// that is SetBadFlag()ed (or replaced by a fused point) stayed allocated for
// good: on the headset LocalMapping inserts and culls keyframes and points
// continuously, and the process grew by ~4.5 MB/s while the map itself stayed
// small. The graveyard frees them once no thread can still use them:
//  - every culled object gets a sequence number when it goes bad (Add);
//  - LocalMapping and LoopClosing report, at a point where they hold no
//    keyframe or map point of their own, the sequence number read before they
//    last fetched any (SetQuiescent);
//  - a reset map (Tracking::ResetActiveMap) used to be dropped whole; its
//    keyframes and map points go here too, and its keyframes are deleted;
//  - keyframes both have passed are released by the tracking thread, holding
//    the map's mMutexMapUpdate (Collect, KeyFrame::ReleaseFeatures). They stay
//    allocated: the trajectory lists, mpReferenceKF and the parent chains of
//    bad keyframes still use their pose. Their features are nearly all of
//    their size;
//  - bad map points are still referenced by keyframes (ORB-SLAM3 checks
//    isBad() on keyframe matches everywhere instead of keeping them clean), so
//    LocalMapping first erases them from every keyframe (TakeForScrub, then
//    Scrubbed). Once both threads have started over after that, nothing can
//    reach them but the tracking thread's frames, which it clears before
//    deleting them (Collect, Tracking::FreeCulled).
class Graveyard
{
public:
    enum Holder { LOCAL_MAPPING = 0, LOOP_CLOSING = 1, HOLDERS = 2 };

    // bDelete: a keyframe of a reset map (Tracking::ResetActiveMap), which
    // nothing refers to any more: delete it instead of releasing it.
    static void Add(KeyFrame* pKF, bool bDelete = false);
    static void Add(MapPoint* pMP);
    // Sequence number of the next event; read it before fetching keyframes or map points.
    static unsigned long long Seq();
    // holder no longer uses any object culled (or scrubbed) before seq.
    static void SetQuiescent(Holder holder, unsigned long long seq);
    // Map points both holders have passed, to erase from all keyframes (vpBadKFs:
    // the culled keyframes not released yet, which keep their matches until then).
    static void TakeForScrub(std::vector<MapPoint*> &vpMPs, std::vector<KeyFrame*> &vpBadKFs);
    static void Scrubbed(const std::vector<MapPoint*> &vpMPs);
    // Keyframes to release or delete and scrubbed map points to delete, culled
    // at least minAgeSeconds ago. No keyframes while LocalMapping scrubs.
    static void Collect(std::vector<KeyFrame*> &vpKFs, std::vector<KeyFrame*> &vpKFsDelete,
                        std::vector<MapPoint*> &vpMPs, double minAgeSeconds);
    // Freed so far, and still waiting.
    static void Stats(unsigned long &nKFsReleased, unsigned long &nMPsDeleted, unsigned long &nPending);
};

} //namespace ORB_SLAM3

#endif // MAP_H
