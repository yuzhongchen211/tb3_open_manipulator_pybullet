import pybullet as p

from Env.world import World

if __name__ == '__main__':
	world = World(urdf_path="./urdf/model.urdf")
	log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "log/robotmove.mp4")
	while True:
		world.keyboard_control()
		world.step(capture_image=True)
		if world.done:
			break
	p.stopStateLogging(log_id)
	world.close()