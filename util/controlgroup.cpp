#include "../include/controlgroup.h"

ControlGroup::ControlGroup()
{
}
ControlGroup::~ControlGroup()
{
}
eros::diagnostic ControlGroup::init(eros::diagnostic t_diag, Mode t_mode, std::string _name)
{
	print_tuning_info = false;
	name = _name;
	initialized = false;
	ready = false;
	run_time = 0.0;
	gain_P = 0.0;
	gain_I = 0.0;
	gain_D = 0.0;
	error = 0.0;
	error_perc = 0.0;
	integral_error = 0.0;
	integral_reset_counter = 0;
	output_max = 0.0;
	output_default = 0.0;
	output_min = 0.0;
	input_max = 0.0;
	input_default = 0.0;
	input_min = 0.0;
	derivative_error = 0.0;
	delta_output_limit = 0.0;
	P_term = 0.0;
	I_term = 0.0;
	D_term = 0.0;
	diagnostic = t_diag;
	diagnostic.DeviceName = _name;
	if (t_mode == ControlGroup::Mode::UNKNOWN)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic.Description = "ControlGroup: " + _name + " Unsupported Mode: UNKNOWN.";
		return diagnostic;
	}
	mode = t_mode;
	return diagnostic;
}
bool ControlGroup::set_PIDGains(double t_P, double t_I, double t_D)
{
	if (mode == ControlGroup::Mode::PID)
	{
		gain_P = t_P;
		gain_I = t_I;
		gain_D = t_D;
		return true;
	}
	else
	{
		return false;
	}
}

eros::diagnostic ControlGroup::update(double dt)
{
	eros::diagnostic diag = diagnostic;
	diag.Level = INFO;
	diag.Diagnostic_Type = ACTUATORS;
	diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
	diag.Description = "No Error";
	run_time += dt;
	bool output_updated = false;
	double out = 0.0;
	if (initialized == true)
	{
		if (ready == false)
		{
			bool all_data_ok = true;
			for (std::size_t i = 0; i < inputs.size(); ++i)
			{
				all_data_ok = all_data_ok && inputs.at(i).properties_initialized;
				if (inputs.at(i).properties_initialized == false)
				{
					diag.Diagnostic_Type = DATA_STORAGE;
					diag.Level = WARN;
					diag.Diagnostic_Message = INITIALIZING;
					diag.Description = "Input: " + inputs.at(i).name + " does not have all properties defined yet.";
					return diag;
				}
			}
			for (std::size_t i = 0; i < outputs.size(); ++i)
			{
				all_data_ok = all_data_ok && outputs.at(i).properties_initialized;
				if (outputs.at(i).properties_initialized == false)
				{
					diag.Diagnostic_Type = DATA_STORAGE;
					diag.Level = WARN;
					diag.Diagnostic_Message = INITIALIZING;
					diag.Description = "Output: " + outputs.at(i).name + " does not have all properties defined yet.";
					return diag;
				}
			}
			if (all_data_ok == true)
			{
				ready = true;
				diag.Diagnostic_Type = DATA_STORAGE;
				diag.Level = NOTICE;
				diag.Diagnostic_Message = NOERROR;
				diag.Description = "ControlGroup: " + name + " Initialized and Ready.";
				return diag;
			}
		}
	}
	if (ready == true)
	{

		double last_error = error;
		double v = 0.0;
		double d_out = 0.0;
		if (mode == Mode::PID)
		{
			bool inputs_ok = false;
			if ((inputs.at(0).rx_counter > 0) && (inputs.at(1).rx_counter > 0))
			{
				inputs_ok = true;
				error = inputs.at(1).signal.value - inputs.at(0).signal.value;
				if (fabs(inputs.at(1).signal.value) < 0.000001)
				{
					error_perc = 0.0;
				}
				else
				{
					error_perc = 100.0 * error / inputs.at(1).signal.value;
				}

				integral_error += (error)*dt;
				if ((inputs.at(0).rx_counter == 1) && (inputs.at(1).rx_counter == 1))
				{
					derivative_error = 0.0;
				}
				else
				{
					derivative_error = (error - last_error) / dt;
				}

				P_term = gain_P * error;
				I_term = gain_I * integral_error;
				D_term = gain_D * derivative_error;
				v = (P_term + I_term + D_term) + output_default;
				d_out = (v - outputs.at(0).signal.value) / dt;
				output_updated = true;
			}
			else if ((inputs.at(0).rx_counter == 0) || (inputs.at(0).rx_counter))
			{
				diag.Level = NOTICE;
				diag.Diagnostic_Type = ACTUATORS;
				diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
				diag.Description = "ControlGroup: " + name +
								   " Has not received all inputs yet: Input 0: " + std::to_string(inputs.at(0).rx_counter) +
								   " Input 1: " + std::to_string(inputs.at(1).rx_counter);
			}
			if (inputs_ok == true)
			{
				if (v > output_max)
				{
					diag.Level = NOTICE;
					diag.Diagnostic_Type = ACTUATORS;
					diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
					diag.Description = "Commanded Output of: " + std::to_string(v) + " Exceeded Max Output Limit of: " + std::to_string(output_max);
					out = output_max;
				}
				else if (v < output_min)
				{
					diag.Level = NOTICE;
					diag.Diagnostic_Type = ACTUATORS;
					diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
					diag.Description = "Commanded Output of: " + std::to_string(v) + " Exceeded Min Output Limit of: " + std::to_string(output_min);
					out = output_min;
				}
				else if (fabs(d_out) > delta_output_limit)
				{
					if (d_out > 0.0)
					{
						out = (delta_output_limit * dt) + outputs.at(0).signal.value;
					}
					else
					{
						out = (-delta_output_limit * dt) + outputs.at(0).signal.value;
					}
					diag.Level = NOTICE;
					diag.Diagnostic_Type = ACTUATORS;
					diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
					diag.Description = "Commanded Delta Output of: " + std::to_string(d_out) +
									   " Exceeded Max Delta Output Limit of: " + std::to_string(delta_output_limit) + " Setting to: " + std::to_string(out);
				}
				else
				{
					out = v;
				}
			}
			if (output_updated == true)
			{
				outputs.at(0).signal.value = out;
				outputs.at(0).pin.Value = (int32_t)out;
				if (outputs.at(0).signal.status == SIGNALSTATE_INITIALIZING)
				{
					outputs.at(0).signal.status = SIGNALSTATE_UPDATED;
				}
			}
			if (print_tuning_info == true)
			{
				printf("CG: %s In1: %4.2f(%d) In2: %4.2f(%d) Err: %4.6f Int Err: %4.2f"
					   " P: %4.2f(%4.2f) I: %4.2f(%4.2f) D: %4.2f(%4.2f) Out: %4.2f,%4.2f,%d\n",
					   name.c_str(),
					   inputs.at(0).signal.value,
					   inputs.at(0).rx_counter,
					   inputs.at(1).signal.value,
					   inputs.at(1).rx_counter,
					   error,
					   integral_error,
					   gain_P,
					   P_term,
					   gain_I,
					   I_term,
					   gain_D,
					   D_term,
					   v,
					   outputs.at(0).signal.value,
					   outputs.at(0).pin.Value);
				print_diagnostic(INFO, diag);
			}
		}
		if (mode == Mode::DIRECT)
		{
			double v = scale_value(inputs.at(0).signal.value, input_default, output_default, input_min, input_max,
								   output_min, output_max, 0.0);
			if (v > output_max)
			{
				diag.Level = NOTICE;
				diag.Diagnostic_Type = ACTUATORS;
				diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
				diag.Description = "Commanded Output of: " + std::to_string(v) + " Exceeded Max Output Limit of: " + std::to_string(output_max);
				out = output_max;
			}
			else if (v < output_min)
			{
				diag.Level = NOTICE;
				diag.Diagnostic_Type = ACTUATORS;
				diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
				diag.Description = "Commanded Output of: " + std::to_string(v) + " Exceeded Min Output Limit of: " + std::to_string(output_min);
				out = output_min;
			}
			else
			{
				out = v;
			}
			outputs.at(0).signal.value = out;
			outputs.at(0).pin.Value = (int32_t)out;
			if ((inputs.at(0).rx_counter > 0) && (outputs.at(0).signal.status == SIGNALSTATE_INITIALIZING))
			{
				outputs.at(0).signal.status = SIGNALSTATE_UPDATED;
			}
		}
		else // No Other Modes Supported Currently
		{
		}
	}
	diagnostic = diag;
	return diagnostic;
}
eros::diagnostic ControlGroup::initialize_signal(SignalDirection dir, std::string name, uint8_t type)
{
	eros::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = DATA_STORAGE;
	eros::signal signal;
	signal.name = name;
	signal.type = type;
	signal.value = input_default;
	signal.status = SIGNALSTATE_INITIALIZING;
	signal.rms = 0.0;
	if (dir == SignalDirection::INPUT)
	{
		Input input;
		input.properties_initialized = true;
		input.signal = signal;
		input.name = name;
		input.signal_class = SignalClass::SIGNAL;
		input.rx_counter = 0;
		inputs.push_back(input);
	}
	else if (dir == SignalDirection::OUTPUT)
	{
		Output output;
		output.properties_initialized = true;
		output.signal = signal;
		output.name = name;
		output.signal_class = SignalClass::SIGNAL;
		outputs.push_back(output);
	}
	else
	{
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "ControlGroup: " + name + " Direction: " + std::to_string((int)dir) + " is not Supported.";
		return diag;
	}
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "ControlGroup: " + name + " Signal: " + name + " initialized.";
	return diag;
}
eros::diagnostic ControlGroup::set_pinproperties(eros::pin pin)
{
	eros::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = DATA_STORAGE;
	bool found = false;
	for (std::size_t i = 0; i < inputs.size(); ++i)
	{
		if ((inputs.at(i).signal_class == SignalClass::PIN))
		{
			if ((inputs.at(i).pin.ConnectedDevice == pin.ConnectedDevice))
			{
				found = true;
				inputs.at(i).properties_initialized = true;
				inputs.at(i).pin = pin;
			}
		}
	}
	if (found == false)
	{
		for (std::size_t i = 0; i < outputs.size(); ++i)
		{
			if ((outputs.at(i).signal_class == SignalClass::PIN))
			{
				if ((outputs.at(i).pin.ConnectedDevice == pin.ConnectedDevice))
				{
					found = true;
					outputs.at(i).properties_initialized = true;
					pin.Value = pin.DefaultValue;
					outputs.at(i).pin = pin;
				}
			}
		}
	}
	if (found == false)
	{
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "ControlGroup: " + name + " Pin: " + pin.ConnectedDevice + " Not Found..";
	}
	else
	{
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "ControlGroup: " + name + " Pin: " + pin.ConnectedDevice + " initialized.";
	}
	return diag;
}
eros::diagnostic ControlGroup::initialize_pin(SignalDirection dir, std::string connected_device)
{
	eros::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = DATA_STORAGE;
	eros::pin pin;
	pin.ConnectedDevice = connected_device;
	std::string signal_name = pin.ConnectedDevice;
	eros::signal signal;
	signal.name = connected_device;
	signal.value = 0.0;
	signal.status = SIGNALSTATE_INITIALIZING;
	signal.rms = 0.0;
	if (dir == SignalDirection::INPUT)
	{
		Input input;
		input.properties_initialized = false;
		input.pin = pin;
		input.name = signal_name;
		input.signal_class = SignalClass::PIN;
		input.signal = signal;
		inputs.push_back(input);
	}
	else if (dir == SignalDirection::OUTPUT)
	{
		Output output;
		output.properties_initialized = false;
		output.pin = pin;
		output.name = signal_name;
		output.signal_class = SignalClass::PIN;
		output.signal = signal;
		outputs.push_back(output);
	}
	else
	{
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "ControlGroup: " + name + " Direction: " + std::to_string((int)dir) + " is not Supported.";
		return diag;
	}
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "ControlGroup: " + name + " Signal: " + signal_name + " initialized.";
	return diag;
}
eros::diagnostic ControlGroup::new_input(std::string sig_name,double value)
{
	eros::diagnostic diag = diagnostic;
	bool found = false;
	for (std::size_t i = 0; i < inputs.size(); ++i)
	{
		if (inputs.at(i).signal.name == sig_name)
		{
			found = true;
			inputs.at(i).signal.value = value;
			inputs.at(i).rx_counter++;
		}
	}
	if (found == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Signal: " + sig_name + " Not Found in Control Group: " + name;
		return diag;
	}
	return diag;
}
eros::diagnostic ControlGroup::new_input(eros::signal signal)
{
	eros::diagnostic diag = diagnostic;
	bool found = false;
	for (std::size_t i = 0; i < inputs.size(); ++i)
	{
		if (inputs.at(i).signal.name == signal.name)
		{
			found = true;
			inputs.at(i).signal = signal;
			inputs.at(i).rx_counter++;
		}
	}
	if (found == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Signal: " + signal.name + " Not Found in Control Group: " + name;
		return diag;
	}
	return diag;
}
double ControlGroup::scale_value(double x, double in_default, double neutral, double x1, double x2, double y1, double y2, double deadband)
{
	int state = 0;
	double out = 0.0;
	double m = 0.0;
	if (x < (in_default))
	{
		state = -1;
		m = (y1 - neutral) / (x1 - in_default);
		out = m * (x - x1) + y1;
	}
	else if (x > in_default)
	{
		state = 1;
		m = (y2 - neutral) / (x2 - in_default);
		out = m * (x - x2) + y2;
	}
	else
	{
		out = neutral;
	}
	//printf("in: %f s: %d m: %f out: %f\n",x,state,m,out);
	return out;
}
eros::diagnostic ControlGroup::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = DATA_STORAGE;
	if (mode == Mode::DIRECT)
	{
		if (inputs.size() != 1)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Description = "ControlGroup: " + name + " Mode: Direct Requires only 1 Input Signals.  Currently Defined: " + std::to_string((int)inputs.size());
			return diag;
		}
		for (std::size_t i = 0; i < inputs.size(); ++i)
		{
			if (inputs.at(i).signal_class != SignalClass::SIGNAL)
			{
				diag.Level = ERROR;
				diag.Diagnostic_Message = INITIALIZING_ERROR;
				diag.Description = "ControlGroup: " + name + " Mode: Direct Requires all Input Signals to be Class: SIGNAL.  Currently: " + inputs.at(i).name + " is Type: " + std::to_string((int)inputs.at(i).signal_class);
				return diag;
			}
			if (inputs.at(i).signal.type == SIGNALTYPE_UNDEFINED)
			{
				diag.Level = ERROR;
				diag.Diagnostic_Message = INITIALIZING_ERROR;
				diag.Description = "ControlGroup: " + name + " Signal: " + inputs.at(i).name + " has unsupported Units.";
			}
		}
		if (outputs.size() != 1)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Description = "ControlGroup: " + name + " Mode: Direct Requires only 1 Output Signals.  Currently Defined: " + std::to_string((int)outputs.size());
			return diag;
		}
		if ((outputs.at(0).signal_class == SignalClass::PIN))
		{
		}
		else
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Description = "ControlGroup: " + name + " Mode: Direct Output Signal Type: " + std::to_string((int)outputs.at(0).signal_class) + " is not Supported.";
			return diag;
		}
	}
	else if (mode == Mode::PID)
	{
		if (inputs.size() != 2)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Description = "ControlGroup: " + name + " Mode: PID Requires only 2 Input Signals.  Currently Defined: " + std::to_string((int)inputs.size());
			return diag;
		}
		for (std::size_t i = 0; i < inputs.size(); ++i)
		{
			if (inputs.at(i).signal_class != SignalClass::SIGNAL)
			{
				diag.Level = ERROR;
				diag.Diagnostic_Message = INITIALIZING_ERROR;
				diag.Description = "ControlGroup: " + name + " Mode: PID Requires all Input Signals to be Class: SIGNAL.  Currently: " + inputs.at(i).name + " is Type: " + std::to_string((int)inputs.at(i).signal_class);
				return diag;
			}
			if (inputs.at(i).signal.type == SIGNALTYPE_UNDEFINED)
			{
				diag.Level = ERROR;
				diag.Diagnostic_Message = INITIALIZING_ERROR;
				diag.Description = "ControlGroup: " + name + " Signal: " + inputs.at(i).name + " has unsupported Units.";
			}
		}
		if (outputs.size() != 1)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Description = "ControlGroup: " + name + " Mode: PID Requires only 1 Output Signals.  Currently Defined: " + std::to_string((int)outputs.size());
			return diag;
		}
		if ((outputs.at(0).signal_class == SignalClass::SIGNAL) ||
			(outputs.at(0).signal_class == SignalClass::PIN))
		{
		}
		else
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Description = "ControlGroup: " + name + " Mode: PID Output Signal Type: " + std::to_string((int)outputs.at(0).signal_class) + " is not Supported.";
			return diag;
		}
	}
	else
	{
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Mode: " + std::to_string((int)mode) + " is not Supported.";
		return diag;
	}
	initialized = true;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "ControlGroup: " + name + " Initialized Successfully.";
	return diag;
}