#ifndef SIMULATE_VR_H_
#define SIMULATE_VR_H_


#include <mujoco/mujoco.h>
#include <openvr.h>
#include <vector>


class mjvrHMD
{
public:
	uint32_t width, height;         // recommended image size per eye

	void initHmd();

	void initTextures(mjvScene& scn);

	void update(mjvScene& scn);

	void render(mjrContext& con, mjuiState& uistate);

	// unused
	void close();

	void transform(mjvScene& scn, mjModel* m);

	void setOBuffer(mjVisual& vis);

	void findVrTransformations(mjModel* m);

	// VR transformations loaded from the model
	int vr_transform_i = -1;
	int vr_transform_inums[mjMAXUIITEM];
	int n_vr_transforms = 0;

	void applyDefaultVrTransform(mjvScene& scn);
	void applyVrTransform(mjvScene& scn, mjModel* m);
	void applyNextVrTransform(mjvScene& scn, mjModel* m);
	void applyPrevVrTransform(mjvScene& scn, mjModel* m);

	bool isInitialized();

private:
	// set to false if not properly initialized or not found hmd
	bool initialized = false;

	// constant properties
	vr::IVRSystem* system;          // opaque pointer returned by VR_Init
	int device_id;                         // hmd device id
	unsigned int idtex;             // OpenGL texture id for Submit
	float eyeoffset[2][3];          // head-to-eye offsets (assume no rotation)

	// pose in room (raw data)
	float roompos[3];               // position
	float roommat[9];               // orientation matrix


	// copy one pose from vr to mjc format
	static void copyPose(const vr::TrackedDevicePose_t* pose, float* roompos, float* roommat);
};


#endif	// SIMULATE_VR_H_
