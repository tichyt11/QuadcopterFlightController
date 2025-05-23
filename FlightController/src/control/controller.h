// #include "algebra.h"
// #include "config.h"
#include "PID.h"
#include "filter.h"

class Controller{
    
	private:
		PID roll_rate_PID;
		PID pitch_rate_PID;
		PID yaw_rate_PID; 
		PID alt_PID;

		float roll_P;
		float pitch_P;
		float yaw_P;

		bool hold_alt;
		float alt_ref;

		Matrix3 DCM;
		Vector4 motor_percentages;
	public:
		Controller();
    	Vector3 last_PID_outputs;
		Control last_reference;

		void update_DCM(Matrix3 DCM);
		Vector4 update_motor_percentages(Control commands, Measurements m);
		Vector4 mix_motors(Vector3 forces, Matrix3 DCM, float throttle, float battery);
		void update_PID_params(int axis, PID_config cfg);

};