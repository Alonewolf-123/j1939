import logging
import time
import can
import j1939

#logging.basicConfig()
logging.getLogger('j1939').setLevel(logging.DEBUG)
logging.getLogger('can').setLevel(logging.DEBUG)

class OwnCaToProduceCyclicMessages(j1939.ControllerApplication):
    """CA to produce messages

    This CA produces simulated sensor values and cyclically sends them to
    the bus with the PGN 0xFEF6 (Intake Exhaust Conditions 1).
    """

    def __init__(self, name, device_address_preferred=None):
        # old fashion calling convention for compatibility with Python2
        j1939.ControllerApplication.__init__(self, name, device_address_preferred)
        self.pgns = []
        self.data_ERC1 = 0
        self.data_EEC1 = 0
        self.data_CSA = 0
        self.data_ETC2 = 0
        self.data_LFC1 = 0
        self.data_LFE1 = 0
        self.data_EEC2 = 0
        self.data_BUSC = 0
        self.data_EEC3 = 0
        self.data_CCSS = 0
        self.data_EEL_P1 = 0
        self.data_IC1 = 0
        self.data_AT1S1 = 0
        self.data_AT1T1_1 = 0
        self.data_HRLFC = 0
        self.data_DPFC1 = 0
        self.data_ET1 = 0
        self.data_FD1 = 0
        self.data_AMB = 0
        self.data_IO = 0
        self.data_OI = 0
        self.data_RC = 0
        self.data_Hours = 0

    def start(self):
        """Starts the CA
        (OVERLOADED function)
        """
        # add our timer event
        self._ecu.add_timer(0.500, self.timer_callback)
        # call the super class function
        return j1939.ControllerApplication.start(self)

    def stop(self):
        """Stops the CA
        (OVERLOADED function)
        """
        self._ecu.remove_timer(self.timer_callback)
        
    def _get_right_bits(self, data, index):
        return data & ( 2 ** index - 1 )
    
    def _get_left_bits(self, data, index):
        return (data) / ( 2 ** index )
    
    def _parse_ERC1(self, data):
        print list(data)
        self.data_ERC1 = data
        data =  int("".join("%02x" % b for b in list(data)), 16)
        retarder_torque_mode = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        retarder_enable_brake = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        retarder_enable_shift = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        actual_retarder = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        actual_retarder = actual_retarder - 125
        intended_retarder = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        intended_retarder = intended_retarder - 125
        engine_coolant_load_increase = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        retarder_requesting_brake_light = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        retarder_road_speed_limit = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        retarder_road_speed_exceeded = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        source_address_of_controlling_device = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        drivers_demand_retarder = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        drivers_demand_retarder = drivers_demand_retarder - 125
        retarder_selection = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_selection = retarder_selection * 0.4
        actual_maximum_available_retarder = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        actual_maximum_available_retarder = actual_maximum_available_retarder - 125
        print("retarder_torque_mode {} retarder_enable_brake {} retarder_enable_shift {} actual_retarder {}% intended_retarder{}% engine_coolant_load_increase{} retarder_requesting_brake_light {} retarder_road_speed_limit {} retarder_road_speed_exceeded {} source_address_of_controlling_device {} drivers_demand_retarder {}% retarder_selection {}% actual_maximum_available_retarder {} %".format(
        retarder_torque_mode, retarder_enable_brake, retarder_enable_shift, actual_retarder, intended_retarder, engine_coolant_load_increase, retarder_requesting_brake_light, retarder_road_speed_limit, retarder_road_speed_exceeded, source_address_of_controlling_device, drivers_demand_retarder, retarder_selection, actual_maximum_available_retarder)) 
        print '=================='
        
    def _parse_EEC1(self, data):
        print list(data)
        self.data_EEC1 = data
        data =  int("".join("%02x" % b for b in list(data)), 16)
        engine_torque_mode = self._get_right_bits(data, 4)
        engine_torque_mode = engine_torque_mode
        data = self._get_left_bits(data, 4)
        actual_engine_fractional = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        actual_engine_fractional = actual_engine_fractional * 0.125
        driver_demand_engine = self._get_right_bits(data, 8)
        driver_demand_engine = driver_demand_engine - 125
        data = self._get_left_bits(data, 8)
        actual_engine = self._get_right_bits(data, 8)
        actual_engine = actual_engine - 125
        data = self._get_left_bits(data, 8)
        engine_speed_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_speed_lsb = self._get_right_bits(data, 8)
        engine_speed = 256 * engine_speed_msb + engine_speed_lsb
        engine_speed = engine_speed * 0.125
        data = self._get_left_bits(data, 8)
        source_address = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_starter = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        engine_demand = self._get_right_bits(data, 8)
        engine_demand = engine_demand - 125
        data = self._get_left_bits(data, 8)
        print("engine_torque_mode {}  actual_engine_fractional {}%  driver_demand_engine {}% actual_engine {}% engine_speed {}rpm source_address {} engine_starter {} engine_demand {}%".format(engine_torque_mode, actual_engine_fractional, driver_demand_engine, actual_engine, engine_speed, source_address, engine_starter, engine_demand))
        print '=================='
        
    def _parse_CSA(self, data):
        print list(data)
        self.data_CSA = data
        data = int("".join("%02x" % b for b in list(data)), 16)
        engine_start_enable_device_1 = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        engine_start_enable_device_2 = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        engine_start_enable_device_1_config = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        engine_start_enable_device_2_config = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        engine_cold_start_fuel_lgniter_cmd = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_cold_start_fuel_lgniter_cmd = engine_cold_start_fuel_lgniter_cmd * 0.4
        engine_cold_start_fuel_lgniter_relay = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        engine_cold_start_fuel_lgniter_relay_feedback = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        print("engine_start_enable_device_1 {} engine_start_enable_device_2 {} engine_start_enable_device_1_config {} engine_start_enable_device_2_config {} engine_cold_start_fuel_lgniter_cmd {} % engine_cold_start_fuel_lgniter_relay_feedback{}".format(
            engine_start_enable_device_1, engine_start_enable_device_2, engine_start_enable_device_1_config, engine_start_enable_device_2_config, engine_cold_start_fuel_lgniter_cmd, engine_cold_start_fuel_lgniter_relay, engine_cold_start_fuel_lgniter_relay_feedback))
        print '=================='
    
    def parse_ETC2(data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_ETC2 = data
        gear = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        gear = gear - 125
        actual_ratio_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        actual_ratio_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        actual_ratio = actual_ratio_msb * 256 + actual_ratio_lsb
        actual_ratio = actual_ratio * 0.001
        gear_engaged = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        gear_engaged = gear_engaged - 125
        range_operator_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        range_operator_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        range_transmission_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        range_transmission_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        print("gear {} actual_ratio {} gear_engaged {} range_operator {0}{0} range_transmission_msb {0}{0}".format(
            gear, actual_ratio, gear_engaged, range_operator_msb, range_operator_lsb, range_transmission_msb, range_transmission_lsb))
        print '=================='
    
    def parse_LFC1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_LFC1 = data
        engine_trip_fuel_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel = 256 * 256 * (256 * engine_trip_fuel_mmsb + engine_trip_fuel_mlsb) + (
            256 * engine_trip_fuel_lmsb + engine_trip_fuel_llsb)
        engine_trip_fuel = engine_trip_fuel * 0.5
        engine_total_fuel_used_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used = 256 * 256 * (256 * engine_total_fuel_used_mmsb + engine_total_fuel_used_mlsb) + (
            256 * engine_total_fuel_used_lmsb + engine_total_fuel_used_llsb)
        engine_total_fuel_used = engine_total_fuel_used * 0.5
        print("engine_trip_fuel {} L engine_total_fuel_used {} L".format(
            engine_trip_fuel, engine_total_fuel_used))
        print '=================='
        
    def parse_LFE1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_LFE1 = data
        engine_fuel_rate_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_fuel_rate_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_fuel_rate = engine_fuel_rate_msb * 256 + engine_fuel_rate_lsb
        engine_fuel_rate = engine_fuel_rate * 0.05
        engine_instaneous_fuel_economy_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_instaneous_fuel_economy_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_instaneous_fuel_economy = engine_instaneous_fuel_economy_msb * \
            256 + engine_instaneous_fuel_economy_lsb
        engine_instaneous_fuel_economy = engine_instaneous_fuel_economy / 512.0
        engine_average_fuel_economy_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_average_fuel_economy_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_average_fuel_economy = engine_average_fuel_economy_msb * \
            256 + engine_average_fuel_economy_lsb
        engine_average_fuel_economy = engine_average_fuel_economy / 512.0
        engine_throttle_valve_1_position = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_throttle_valve_1_position = engine_throttle_valve_1_position * 0.4
        engine_throttle_valve_2_position = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_throttle_valve_2_position = engine_throttle_valve_2_position * 0.4
        print("engine_fuel_rate {} L/h engine_instaneous_fuel_economy {} km/L engine_average_fuel_economy {} km/h engine_throttle_valve_1_position {} % engine_throttle_valve_2_position {} %".format(
            engine_fuel_rate, engine_instaneous_fuel_economy, engine_average_fuel_economy, engine_throttle_valve_1_position, engine_throttle_valve_2_position))
        print '=================='
        
    def parse_EEC2(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_EEC2 = data
        accelerator_pedal_1_low_idle_switch = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        accelerator_pedal_kickdown_switch = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        road_speed_limit_status = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        accelerator_pedal_2_low_idle_switch = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        accelerator_pedal_position_1 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        accelerator_pedal_position_1 = accelerator_pedal_position_1 * 0.4
        engine_percent_load_at_current_speed = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        remote_accelerator_pedal_position = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        remote_accelerator_pedal_position = remote_accelerator_pedal_position * 0.4
        accelerator_pedal_position_2 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        accelerator_pedal_position_2 = accelerator_pedal_position_2 * 0.4
        vehicle_acceleration_rate_limit_status = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        momentary_engine_maximum_power_enable_feedback = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        dpf_thermal_management_active = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        scr_thermal_management_active = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        actual_maximum_available_engine = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        actual_maximum_available_engine = actual_maximum_available_engine * 0.4
        estimated_pumping = self._get_right_bits(data, 8)
        estimated_pumping = estimated_pumping - 125
        data = self._get_left_bits(data, 8)
        print("accelerator_pedal_1_low_idle_switch {} accelerator_pedal_kickdown_switch {} road_speed_limit_status {} accelerator_pedal_2_low_idle_switch {} accelerator_pedal_position_1 {} % engine_percent_load_at_current_speed {} % remote_accelerator_pedal_position {} % accelerator_pedal_position_2 {} % vehicle_acceleration_rate_limit_status {} momentary_engine_maximum_power_enable_feedback {} dpf_thermal_management_active {} scr_thermal_management_active {} actual_maximum_available_engine{} % estimated_pumping {} %".format(
            accelerator_pedal_1_low_idle_switch, accelerator_pedal_kickdown_switch, road_speed_limit_status, accelerator_pedal_2_low_idle_switch, accelerator_pedal_position_1, engine_percent_load_at_current_speed, remote_accelerator_pedal_position, accelerator_pedal_position_2, vehicle_acceleration_rate_limit_status, momentary_engine_maximum_power_enable_feedback, dpf_thermal_management_active, scr_thermal_management_active, actual_maximum_available_engine, estimated_pumping))
        print '=================='
        
    def parse_BUSC(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_BUSC = data
        dead_bus = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        pahse_match = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        frequency_match = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        voldate_match = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        in_sync = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        ac_phase_difference_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        ac_phase_difference_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        ac_phase_difference = ac_phase_difference_lsb + 256 * ac_phase_difference_msb
        ac_phase_difference = ac_phase_difference / 128 - 200
        print("dead_bus {} pahse_match {} frequency_match {} voldate_match {} in_sync {} ac_phase_difference {} deg".format(
            dead_bus, pahse_match, frequency_match, voldate_match, in_sync, ac_phase_difference))

        print '=================='
        
    def parse_EEC3(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_EEC3 = data
        normal_friction = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        normal_friction = normal_friction - 125
        engine_desired_operating_speed_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_desired_operating_speed_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_desired_operating_speed = engine_desired_operating_speed_lsb + \
            engine_desired_operating_speed_msb * 256
        engine_desired_operating_speed = engine_desired_operating_speed * 0.125
        engine_desired_operating_speed_asymmetry = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        estimated_engine_parasitic_losses = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        estimated_engine_parasitic_losses = estimated_engine_parasitic_losses - 125
        exhaust_gas_mass_flow_rate_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        exhaust_gas_mass_flow_rate_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        exhaust_gas_mass_flow_rate = 256 * \
            exhaust_gas_mass_flow_rate_msb + exhaust_gas_mass_flow_rate_lsb
        exhaust_gas_mass_flow_rate = exhaust_gas_mass_flow_rate * 0.2
        intake_dew_point = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        exhaust_dew_point = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        intake_dew_point_2 = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        exhaust_dew_point_2 = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        print("normal_friction {}% engine_desired_operating_speed {} rpm engine_desired_operating_speed_asymmetry {} estimated_engine_parasitic_losses {} % exhaust_gas_mass_flow_rate {} kg/h intake_dew_point {} exhaust_dew_point {} intake_dew_point_2 {} exhaust_dew_point_2 {}".format(
            normal_friction, engine_desired_operating_speed, engine_desired_operating_speed_asymmetry, estimated_engine_parasitic_losses, exhaust_gas_mass_flow_rate, intake_dew_point, exhaust_dew_point, intake_dew_point_2, exhaust_dew_point_2)) 
        print '=================='
        
    def parse_CCSS(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_CCSS = data
        maximum_vehicle_speed_limit = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        cruise_control_high_set_limit_speed = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        cruise_control_low_set_limit_speed = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        maximum_vehicle_speed_list_high_resolution_msb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        maximum_vehicle_speed_list_high_resolution_lsb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        maximum_vehicle_speed_list_high_resolution = 256 * \
            maximum_vehicle_speed_list_high_resolution_msb + \
            maximum_vehicle_speed_list_high_resolution_lsb
        maximum_vehicle_speed_list_high_resolution = maximum_vehicle_speed_list_high_resolution * \
            (1 / 256.0)
        print("maximum_vehicle_speed_limit {}hm/h cruise_control_high_set_limit_speed {} km/h cruise_control_low_set_limit_speed {} km/h maximum_vehicle_speed_list_high_resolution {} km/h".format(
            maximum_vehicle_speed_limit, cruise_control_high_set_limit_speed, cruise_control_low_set_limit_speed, maximum_vehicle_speed_list_high_resolution))   
        print '=================='
        
    def parse_EEL_P1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_EEL_P1 = data
        fuel_delivery_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        fuel_delivery_pressure = fuel_delivery_pressure * 4
        engine_expended_crankcase_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_expended_crankcase_pressure = engine_expended_crankcase_pressure * 0.05
        engine_oil_level = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_oil_level = engine_oil_level * 0.4
        engine_oil_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_oil_pressure = engine_oil_pressure * 4
        engine_crankcase_pressure_1_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_crankcase_pressure_1_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_crankcase_pressure_1 = engine_crankcase_pressure_1_msb * \
            256 + engine_crankcase_pressure_1_lsb
        engine_crankcase_pressure_1 = engine_crankcase_pressure_1 / 128.0
        engine_crankcase_pressure_1 = engine_crankcase_pressure_1 - 250
        engine_coolant_pressure_1 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_coolant_pressure_1 = engine_coolant_pressure_1 * 2
        engine_coolant_level_1 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_coolant_level_1 = engine_coolant_level_1 * 0.4
        print("fuel_delivery_pressure {} kPa engine_expended_crankcase_pressure {} kPa engine_oil_level {} % engine_oil_pressure {} kPa engine_crankcase_pressure_1 {} kPa engine_coolant_pressure_1 {} kPa engine_coolant_level_1 {} %".format(
            fuel_delivery_pressure, engine_expended_crankcase_pressure, engine_oil_level, engine_oil_pressure, engine_crankcase_pressure_1, engine_coolant_pressure_1, engine_coolant_level_1))
        print '=================='
        
    def parse_IC1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_IC1 = data
        diesel_particulate_filter_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_pressure = diesel_particulate_filter_pressure * 0.5
        engine_intake_manifold_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_intake_manifold_pressure = engine_intake_manifold_pressure * 2
        engine_intake_manifold_temperature = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_intake_manifold_temperature = engine_intake_manifold_temperature - 40
        engine_intake_air_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_intake_air_pressure = engine_intake_air_pressure * 2
        engine_air_filter_differential_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_air_filter_differential_pressure = engine_air_filter_differential_pressure * 0.05
        engine_exhaust_temperature_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_exhaust_temperature_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_exhaust_temperature = engine_exhaust_temperature_msb * \
            256 + engine_exhaust_temperature_lsb
        engine_exhaust_temperature = engine_exhaust_temperature * 0.0315 - 273
        engine_coolant_filter_differential_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_coolant_filter_differential_pressure = engine_coolant_filter_differential_pressure * 0.5
        print("diesel_particulate_filter_pressure {} kPa engine_intake_manifold_pressure {} kPa engine_intake_manifold_temperature {} degree engine_intake_air_pressure {} kPa engine_air_filter_differential_pressure {} kPa engine_exhaust_temperature {} degree engine_coolant_filter_differential_pressure {} kPa".format(
            diesel_particulate_filter_pressure, engine_intake_manifold_pressure, engine_intake_manifold_temperature, engine_intake_air_pressure, engine_air_filter_differential_pressure, engine_exhaust_temperature, engine_coolant_filter_differential_pressure))
        print '=================='
        
    def parse_AT1S1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_AT1S1 = data
        diesel_particulate_filter_soot_load = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_ash_load = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_time_since_last_lmsb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_time_since_last_llsb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_time_since_last_mmsb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_time_since_last_mlsb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_time_since_last = 256 * 256 * (diesel_particulate_filter_time_since_last_mmsb * 256 + diesel_particulate_filter_time_since_last_mlsb) + (
            256 * diesel_particulate_filter_time_since_last_lmsb + diesel_particulate_filter_time_since_last_llsb)
        diesel_particulate_filter_soot_load_msb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_soot_load_lsb = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        diesel_particulate_filter_soot_load_resolution = 256 * \
            diesel_particulate_filter_soot_load_msb + \
            diesel_particulate_filter_soot_load_lsb
        diesel_particulate_filter_soot_load_resolution = diesel_particulate_filter_soot_load_resolution * 0.0025
        print("diesel_particulate_filter_soot_load {} % diesel_particulate_filter_ash_load {} % diesel_particulate_filter_time_since_last {} s diesel_particulate_filter_soot_load_resolution {} %".format(
            diesel_particulate_filter_soot_load, diesel_particulate_filter_ash_load, diesel_particulate_filter_time_since_last, diesel_particulate_filter_soot_load_resolution))
        print '=================='
        
    def parse_AT1T1_1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_AT1T1_1 = data
        diesel_exhaust_fluid_tank_volumn = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_exhaust_fluid_tank_volumn = diesel_exhaust_fluid_tank_volumn * 0.4
        diesel_exhaust_fulid_tank_temperature = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_exhaust_fulid_tank_temperature = diesel_exhaust_fulid_tank_temperature - 40
        diesel_exhaust_fulid_tank_level_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_exhaust_fulid_tank_level_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_exhaust_fulid_tank_level = diesel_exhaust_fulid_tank_level_lsb + \
            256 * diesel_exhaust_fulid_tank_level_msb
        diesel_exhaust_fulid_tank_level = diesel_exhaust_fulid_tank_level * 0.1
        diesel_exhaust_fulid_tank_level_volumn = self._get_right_bits(data, 5)
        data = self._get_left_bits(data, 5)
        selective_catalytic_reduction_operator = self._get_right_bits(data, 3)
        data = self._get_left_bits(data, 3)
        diesel_exhaust_fulid_tank_1_temperature = self._get_right_bits(data, 5)
        data = self._get_left_bits(data, 5)
        scr_operator_inducement_severity = self._get_right_bits(data, 3)
        data = self._get_left_bits(data, 3)
        diesel_exhaust_fluid_tank_heater = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        diesel_exhaust_fluid_tank_1_heater = self._get_right_bits(data, 5)
        data = self._get_left_bits(data, 5)
        print("diesel_exhaust_fluid_tank_volumn {} % diesel_exhaust_fulid_tank_temperature {} degree diesel_exhaust_fulid_tank_level {} mm diesel_exhaust_fulid_tank_level_volumn {} selective_catalytic_reduction_operator {} diesel_exhaust_fulid_tank_1_temperature {} scr_operator_inducement_severity {} diesel_exhaust_fluid_tank_heater {} % diesel_exhaust_fluid_tank_1_heater {} %".format(
            diesel_exhaust_fluid_tank_volumn, diesel_exhaust_fulid_tank_temperature, diesel_exhaust_fulid_tank_level, diesel_exhaust_fulid_tank_level_volumn, selective_catalytic_reduction_operator, diesel_exhaust_fulid_tank_1_temperature, scr_operator_inducement_severity, diesel_exhaust_fluid_tank_heater, diesel_exhaust_fluid_tank_1_heater))
        print '=================='
        
    def parse_HRLFC(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_HRLFC = data
        engine_trip_fuel_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_trip_fuel = 256 * 256 * ( 256 * engine_trip_fuel_mmsb + engine_trip_fuel_mlsb ) + 256 * engine_trip_fuel_lmsb + engine_trip_fuel_llsb
        engine_trip_fuel = engine_trip_fuel * 0.001
        engine_total_fuel_used_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_fuel_used = 256 * 256 * (256 * engine_total_fuel_used_mmsb + engine_total_fuel_used_mlsb) + \
            256 * engine_total_fuel_used_lmsb + engine_total_fuel_used_llsb
        engine_total_fuel_used = engine_total_fuel_used * 0.001
        print("engine_trip_fuel {} L engine_total_fuel_used {} L".format(
            engine_trip_fuel, engine_total_fuel_used
        ))
        print '=================='
        
    def parse_DPFC1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_DPFC1 = data
        aftertreatment_diesel_particulate_filter_active = self._get_right_bits(
        data, 2)
        data = self._get_left_bits(data, 2)
        aftertreatment_diesel_particulate_filter_status = self._get_right_bits(
            data, 3)
        data = self._get_left_bits(data, 3)
        diesel_particulate_filter_active_regeneration_inhibited = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_inhibit_switch = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_clutch_disengaged = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_service_brake_active = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_pto_active = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_accelerator_pedal_off_idle = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_out_neutral = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_duevehicle_speed_above_allowed_speed = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_parking_brake_not_set = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_low_exhaust_temperature = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_system_fault_active = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_system_timeout = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_duetemporary_system_lockout = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_permanent_system_locakout = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_engine_now_warmed_up = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due_vehicle_speed_below_allowed_speed = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_automatic_active_regeneration = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        exhaust_system_high_temperature_lamp_command = self._get_right_bits(
            data, 3)
        data = self._get_left_bits(data, 3)
        diesel_particulate_filter_active_regeneration_forced = self._get_right_bits(
            data, 3)
        data = self._get_left_bits(data, 3)
        hydrocarbon_doser_purging_enable = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)
        diesel_particulate_filter_active_regeneration_inhibited_due = self._get_right_bits(
            data, 2)
        data = self._get_left_bits(data, 2)

        print("aftertreatment_diesel_particulate_filter_active {} aftertreatment_diesel_particulate_filter_status {} diesel_particulate_filter_active_regeneration_inhibited {} diesel_particulate_filter_active_regeneration_inhibited_due_inhibit_switch {} diesel_particulate_filter_active_regeneration_inhibited_due_clutch_disengaged {} diesel_particulate_filter_active_regeneration_inhibited_due_service_brake_active {} diesel_particulate_filter_active_regeneration_inhibited_due_pto_active {} diesel_particulate_filter_active_regeneration_inhibited_due_accelerator_pedal_off_idle {} diesel_particulate_filter_active_regeneration_inhibited_due_out_neutral {} diesel_particulate_filter_active_regeneration_inhibited_duevehicle_speed_above_allowed_speed {} diesel_particulate_filter_active_regeneration_inhibited_due_parking_brake_not_set {} diesel_particulate_filter_active_regeneration_inhibited_due_low_exhaust_temperature {} diesel_particulate_filter_active_regeneration_inhibited_due_system_fault_active {} diesel_particulate_filter_active_regeneration_inhibited_due_system_timeout {} diesel_particulate_filter_active_regeneration_inhibited_duetemporary_system_lockout {} diesel_particulate_filter_active_regeneration_inhibited_due_permanent_system_locakout {} diesel_particulate_filter_active_regeneration_inhibited_due_engine_now_warmed_up {} diesel_particulate_filter_active_regeneration_inhibited_due_vehicle_speed_below_allowed_speed {} diesel_particulate_filter_automatic_active_regeneration {} exhaust_system_high_temperature_lamp_command {} diesel_particulate_filter_active_regeneration_forced {} hydrocarbon_doser_purging_enable {} diesel_particulate_filter_active_regeneration_inhibited_due {}".format(
            aftertreatment_diesel_particulate_filter_active, aftertreatment_diesel_particulate_filter_status, diesel_particulate_filter_active_regeneration_inhibited, diesel_particulate_filter_active_regeneration_inhibited_due_inhibit_switch, diesel_particulate_filter_active_regeneration_inhibited_due_clutch_disengaged, diesel_particulate_filter_active_regeneration_inhibited_due_service_brake_active, diesel_particulate_filter_active_regeneration_inhibited_due_pto_active, diesel_particulate_filter_active_regeneration_inhibited_due_accelerator_pedal_off_idle, diesel_particulate_filter_active_regeneration_inhibited_due_out_neutral, diesel_particulate_filter_active_regeneration_inhibited_duevehicle_speed_above_allowed_speed, diesel_particulate_filter_active_regeneration_inhibited_due_parking_brake_not_set, diesel_particulate_filter_active_regeneration_inhibited_due_low_exhaust_temperature, diesel_particulate_filter_active_regeneration_inhibited_due_system_fault_active, diesel_particulate_filter_active_regeneration_inhibited_due_system_timeout, diesel_particulate_filter_active_regeneration_inhibited_duetemporary_system_lockout, diesel_particulate_filter_active_regeneration_inhibited_due_permanent_system_locakout, diesel_particulate_filter_active_regeneration_inhibited_due_engine_now_warmed_up, diesel_particulate_filter_active_regeneration_inhibited_due_vehicle_speed_below_allowed_speed, diesel_particulate_filter_automatic_active_regeneration, exhaust_system_high_temperature_lamp_command, diesel_particulate_filter_active_regeneration_forced, hydrocarbon_doser_purging_enable, diesel_particulate_filter_active_regeneration_inhibited_due))
        print '=================='
        
    def parse_ET1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_ET1 = data
        engine_coolant_temperature = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_coolant_temperature = engine_coolant_temperature - 40
        engine_fuel_temperature = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_fuel_temperature = engine_fuel_temperature - 40
        engine_oil_temperature_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_oil_temperature_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_oil_temperature = engine_oil_temperature_lsb + \
            256 * engine_oil_temperature_msb
        engine_oil_temperature = engine_oil_temperature * 0.03125 - 273
        engine_turbocharger_temperature_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_turbocharger_temperature_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_turbocharger_temperature = engine_turbocharger_temperature_lsb + \
            256 * engine_turbocharger_temperature_msb
        engine_turbocharger_temperature = engine_turbocharger_temperature * 0.03125 - 273
        engine_intercooler_temperature = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_intercooler_temperature = engine_intercooler_temperature - 40
        engine_charge_air_cooler_thermostart_opening = self._get_right_bits(
            data, 8)
        data = self._get_left_bits(data, 8)
        engine_charge_air_cooler_thermostart_opening = engine_charge_air_cooler_thermostart_opening * 0.4
        print("engine_coolant_temperature {} degree engine_fuel_temperature {} degree engine_oil_temperature {} degree engine_turbocharger_temperature {} degree engine_intercooler_temperature {} degree engine_charge_air_cooler_thermostart_opening {} %".format(
            engine_coolant_temperature, engine_fuel_temperature, engine_oil_temperature, engine_turbocharger_temperature, engine_intercooler_temperature, engine_charge_air_cooler_thermostart_opening))
        print '=================='
        
    def parse_FD1(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_FD1 = data
        engine_fan_estimated_percent_speed = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_fan_estimated_percent_speed = engine_fan_estimated_percent_speed * 0.4
        fan_drive_state = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        fan_speed_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        fan_speed_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        fan_speed = 256 * fan_speed_msb + fan_speed_lsb
        fan_speed = fan_speed * 0.125
        hydraulic_fan_motor_pressure_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        hydraulic_fan_motor_pressure_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        hydraulic_fan_motor_pressure = 256 * \
            hydraulic_fan_motor_pressure_msb + hydraulic_fan_motor_pressure_lsb
        hydraulic_fan_motor_pressure = hydraulic_fan_motor_pressure * 0.5
        fan_drive_bypass_command_status = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        fan_drive_bypass_command_status = fan_drive_bypass_command_status * 0.4
        print("engine_fan_estimated_percent_speed {} % fan_drive_state {} fan_speed {} rpm hydraulic_fan_motor_pressure {} kPa fan_drive_bypass_command_status {} %".format(
            engine_fan_estimated_percent_speed, fan_drive_state, fan_speed, hydraulic_fan_motor_pressure, fan_drive_bypass_command_status))
        print '=================='
        
    def parse_AMB(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_AMB = data
        barometric_pressure = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        barometric_pressure = barometric_pressure * 0.5
        cab_interior_temperature_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        cab_interior_temperature_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        cab_interior_temperature = cab_interior_temperature_lsb + 256 * cab_interior_temperature_msb
        cab_interior_temperature = cab_interior_temperature * 0.03125
        cab_interior_temperature = cab_interior_temperature - 273
        ambient_air_temperature_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        ambient_air_temperature_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        ambient_air_temperature = ambient_air_temperature_lsb + \
            256 * ambient_air_temperature_msb
        ambient_air_temperature = ambient_air_temperature * 0.03125
        ambient_air_temperature = ambient_air_temperature - 273
        engine_intake_air_temperature = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_intake_air_temperature = engine_intake_air_temperature - 40
        road_surface_temperature_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        road_surface_temperature_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        road_surface_temperature = road_surface_temperature_lsb + \
            256 * road_surface_temperature_msb
        road_surface_temperature = road_surface_temperature * 0.03125
        road_surface_temperature = road_surface_temperature - 273
        print("barometric_pressure {} kPa cab_interior_temperature {} degree ambient_air_temperature {} degree engine_intake_air_temperature {} degree road_surface_temperature {} degree".format(
            barometric_pressure, cab_interior_temperature, ambient_air_temperature, engine_intake_air_temperature, road_surface_temperature))
        print '=================='
        
    def parse_IO(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_IO = data
        engine_total_idel_fuel_used_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_fuel_used_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_fuel_used_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_fuel_used_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_fuel_used = 256 * 256 * (256 * engine_total_idel_fuel_used_mmsb + engine_total_idel_fuel_used_mlsb) + \
            256 * engine_total_idel_fuel_used_lmsb + engine_total_idel_fuel_used_llsb
        engine_total_idel_fuel_used = engine_total_idel_fuel_used * 0.5
        engine_total_idel_hours_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_hours_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_hours_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_hours_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_idel_hours = 256 * 256 * (256 * engine_total_idel_hours_mmsb + engine_total_idel_hours_mlsb) + \
            256 * engine_total_idel_hours_lmsb + engine_total_idel_hours_llsb
        engine_total_idel_hours = engine_total_idel_hours * 0.05
        print("engine_total_idel_fuel_used {} L engine_total_idel_hours {} h".format(
            engine_total_idel_fuel_used, engine_total_idel_hours
        ))
        print '=================='
        
    def parse_OI(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_OI = data
        water_in_fuel_indicator_1 = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        operator_shift_prompt = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        water_in_fuel_indicator_2 = self._get_right_bits(data, 2)
        data = self._get_left_bits(data, 2)
        driver_warning_system_indicator_status = self._get_right_bits(data, 3)
        data = self._get_left_bits(data, 3)
        emission_control_system_operator_inducement_severity = self._get_right_bits(data, 3)
        data = self._get_left_bits(data, 3)
        print("water_in_fuel_indicator_1 {} operator_shift_prompt {} water_in_fuel_indicator_2 {} driver_warning_system_indicator_status {} emission_control_system_operator_inducement_severity {}".format(
            water_in_fuel_indicator_1, operator_shift_prompt, water_in_fuel_indicator_2, driver_warning_system_indicator_status, emission_control_system_operator_inducement_severity
        ))
        print '=================='
        
    def parse_RC(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_RC = data
        retarder_type = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        retarder_location = self._get_right_bits(data, 4)
        data = self._get_left_bits(data, 4)
        retarder_control_method = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speet_at_idel_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speet_at_idel_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speet_at_idel = retarder_speet_at_idel_msb * 256 + retarder_speet_at_idel_lsb
        retarder_speet_at_idel = retarder_speet_at_idel * 0.125
        retarder_percent_torque_idel_point = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_percent_torque_idel_point = retarder_percent_torque_idel_point - 125
        maximum_retarder_speed_point_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        maximum_retarder_speed_point_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        maximum_retarder_speed_point = 256 * maximum_retarder_speed_point_msb + maximum_retarder_speed_point_lsb
        maximum_retarder_speed_point = maximum_retarder_speed_point * 0.125
        retarder_percent_torque_at_maximum_speed = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_percent_torque_at_maximum_speed = retarder_percent_torque_at_maximum_speed - 125
        retarder_speed_point_3_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speed_point_3_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speed_point_3 = 256 * \
            retarder_speed_point_3_msb + retarder_speed_point_3_lsb
        retarder_speed_point_3 = retarder_speed_point_3 * 0.125
        retarder_percent_torque_point_3 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_percent_torque_point_3 = retarder_percent_torque_point_3 - 125
        retarder_speed_point_4_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speed_point_4_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_speed_point_4 = 256 * \
            retarder_speed_point_4_msb + retarder_speed_point_4_lsb
        retarder_speed_point_4 = retarder_speed_point_4 * 0.125
        retarder_percent_torque_point_4 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_percent_torque_point_4 = retarder_percent_torque_point_4 - 125
        speed_peak_torque_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        speed_peak_torque_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        speed_peak_torque = 256 * \
            speed_peak_torque_msb + speed_peak_torque_lsb
        speed_peak_torque = speed_peak_torque * 0.125
        reference_msb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        reference_lsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        reference = 256 * \
            reference_msb + reference_lsb
        retarder_percent_torque_point_5 = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        retarder_percent_torque_point_5 = retarder_percent_torque_point_5 - 125
        print("retarder_type {} retarder_location {} retarder_control_method {} steps retarder_speet_at_idel {} rpm retarder_percent_torque_idel_point {} % maximum_retarder_speed_point {} rpm retarder_percent_torque_at_maximum_speed {} % retarder_speed_point_3 {} rpm retarder_percent_torque_point_3 {} % retarder_speed_point_4 {} rpm retarder_percent_torque_point_4 {} % speed_peak_torque {} rpm reference {} Nm retarder_percent_torque_point_5 {} %".format(
            retarder_type, retarder_location, retarder_control_method, retarder_speet_at_idel, retarder_percent_torque_idel_point, maximum_retarder_speed_point, retarder_percent_torque_at_maximum_speed, retarder_speed_point_3, retarder_percent_torque_point_3, retarder_speed_point_4, retarder_percent_torque_point_4, speed_peak_torque, reference, retarder_percent_torque_point_5
        ))
        print '=================='
        
    def parse_Hours(self, data):
        print list(data)
        data = int("".join("%02x" % b for b in list(data)), 16)
        self.data_Hours = data
        engine_total_hours_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_hours_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_hours_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_hours_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_hours = 256 * 256 * (256 * engine_total_hours_mmsb + engine_total_hours_mlsb) + (
            256 * engine_total_hours_lmsb + engine_total_hours_llsb)
        engine_total_hours = engine_total_hours * 0.05
        engine_total_revolutions_lmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_revolutions_llsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_revolutions_mmsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_revolutions_mlsb = self._get_right_bits(data, 8)
        data = self._get_left_bits(data, 8)
        engine_total_revolutions = 256 * 256 * (256 * engine_total_revolutions_mmsb + engine_total_revolutions_mlsb) + (
            256 * engine_total_revolutions_lmsb + engine_total_revolutions_llsb)
        engine_total_revolutions = engine_total_revolutions * 1000
        print("engine_total_hours {} h engine_total_revolutions {} r".format(
            engine_total_hours, engine_total_revolutions
        ))
        print '=================='
    
    def on_message(self, pgn, data):
        """Feed incoming message to this CA.
        (OVERLOADED function)
        :param int pgn:
            Parameter Group Number of the message
        :param bytearray data:
            Data of the PDU
        """
        if not (pgn in self.pgns):
            self.pgns.append(pgn)
        #Electronic Engine Controller 1
        temp = pgn
        pgn = 0
        if(pgn == 61444 and self.data_EEC1 != data):
            self._parse_EEC1(data)
            #print("Electronic-Engine-Controller-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), data))
        #Electronic Retarder Controller 1
        if(pgn == 61440 and self.data_ERC1 != data):
            self._parse_ERC1(data)
            #print("Electronic-Retarder-Controller-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Cold Start Aids
        if(pgn == 64966 and self.data_CSA != data):
            self._parse_CSA(data)
            #print("Cold-Start-Aids-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Electronic Elgine Controller2
        if(pgn == 61445 and self.data_ETC2 != data):
            self.parse_ETC2(data)
            #print("Electronic-Engine-Controller-2-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Fuel Consumption (Liquid) 1
        if(pgn == 65257 and self.data_LFC1 != data):
            self.parse_LFC1(data)
            #print("Fule-Consumption-Liquid-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Fuel-Economy-Liquid
        if(pgn == 65266 and self.data_LFE1 != data):
            self.parse_LFE1(data)
            #print("Fuel-Liquid-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Electronic Engine Controller 2
        if(pgn == 61443 and self.data_EEC2 != data):
            self.parse_EEC2(data)
            #print("Electronic-Engine-Controller-2PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Maximum Vehicle Speed Limit Status
        if(pgn == 64997 and self.data_BUSC != data):
            self.parse_BUSC(data)
            #print("Maximum-Vehicle-Speed-List-Status-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Electronic Engine Controller 3
        if(pgn == 65247 and self.data_EEC3 != data):
            self.parse_EEC3(data)
            #print("Electronic-Engine-Controller-3-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Cruise Control/Vehicle Speed Setup
        if(pgn == 65261 and self.data_CCSS != data):
            self.parse_CCSS(data)
            #print("Cruise-Control-Vehicle-Speed-Setup-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Engine Fluid Level/Pressure 1
        if(pgn == 65263 and self.data_EEL_P1 != data):
            self.parse_EEL_P1(data)
            #print("Engine-Fluid-Level-Pressure-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Intake/Exhaust Conditions 1
        if(pgn == 65270 and self.data_IC1 != data):
            self.parse_IC1(data)
            #print("Intake-Exhaust-Conditions-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Aftertreatment Conditions 1
        if(pgn == 64891 and self.data_AT1S1 != data):
            self.parse_AT1S1(data)
            #print("Aftertreatment-Conditions-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Aftertreatment 1 Diesel Exhaust Fluid Tank 1 Information 1
        if(pgn == 65110 and self.data_AT1T1_1 != data):
            self.parse_AT1T1_1(data)
            #print("Aftertreatment-1-Diesel-Exhaust-Flid-Tank-1-Information-1-Level-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Diesel Particulate Filter Control 1
        if(pgn == 64892 and self.data_DPFC1 != data):
            self.parse_DPFC1(data)
            #print("Diesel-Particulate-Filter-Control-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Engine Temperature 1
        pgn = temp
        if(pgn == 65262 and self.data_ET1 != data):
            self.parse_ET1(data)
            #print("Engine-temperature-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #High Resolution Fuel Consumption (Liquid)
        pgn = 0
        if(pgn == 64777 and self.data_HRLFC != data):
            self.parse_HRLFC(data)
            #print("High-Resolution-Fuel-Consumption-Liquid-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Fan Drive 1
        pgn = temp
        if(pgn == 65213 and self.data_FD1 != data):
            self.parse_FD1(data)
            #print("Fan-Drive-1-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Ambient Conditions
        pgn = 0
        pgn = temp
        if(pgn == 65269 and self.data_AMB != data):
            self.parse_AMB(data)
            #print("Ambient-Conditions-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Idle Operation
        pgn = 0
        if(pgn == 65244 and self.data_IO != data):
            self.parse_IO(data)
            #print("Idel-Operation-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Active Diagnostic Trouble Codes
        if(pgn == 65266):
            print("Active-Diagnostic-Trouble-Codes-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Operator indicators
        if(pgn == 65279 and self.data_OI != data):
            self.parse_OI(data)
            #print("Active-Diagnostic-Trouble-Codes-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Electronic Retarder Controller 1
        if(pgn == 65249 and self.data_RC != data):
            self.parse_RC(data)
            #print("Electronic-Retarder-Controller-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))
        #Engine Hours Resolutions
        if(pgn == 65253 and self.data_Hours != data):
            self.parse_Hours(data)
            #print("Engine-Hours-Resolutions-PGN {} length {}  packet {}".format(format(pgn, 'x'), len(data), list(data)))

    def timer_callback(self, cookie):
        """Callback for sending the IEC1 message

        This callback is registered at the ECU timer event mechanism to be 
        executed every 500ms.

        :param cookie:
            A cookie registered at 'add_timer'. May be None.
        """
        # wait until we have our device_address
        if self.state != j1939.ControllerApplication.State.NORMAL:
            # returning true keeps the timer event active
            return True

        pgn = j1939.ParameterGroupNumber(0, 0xFE, 0xCA)
        data = [
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_8, # Particulate Trap Inlet Pressure (SPN 81)
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_8, # Boost Pressure (SPN 102)
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_8, # Intake Manifold 1 Temperature (SPN 105)
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_8, # Air Inlet Pressure (SPN 106)
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_8, # Air Filter 1 Differential Pressure (SPN 107)
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_16_ARR[0], # Exhaust Gas Temperature (SPN 173)
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_16_ARR[1],
            j1939.ControllerApplication.FieldValue.NOT_AVAILABLE_8, # Coolant Filter Differential Pressure (SPN 112)
            ]

        # SPN 105, Range -40..+210
        # (Offset -40)
        receiverTemperature = 30
        data[2] = receiverTemperature + 40

        self.send_message(6, pgn.value, data)

        # returning true keeps the timer event active
        return True


def main():
    print("Initializing")

    # create the ElectronicControlUnit (one ECU can hold multiple ControllerApplications)
    ecu = j1939.ElectronicControlUnit()

    # Connect to the CAN bus
    # Arguments are passed to python-can's can.interface.Bus() constructor
    # (see https://python-can.readthedocs.io/en/stable/bus.html).
    ecu.connect(bustype='socketcan', channel='can0')
    # ecu.connect(bustype='kvaser', channel=0, bitrate=250000)
    #     ecu.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
    # ecu.connect(bustype='ixxat', channel=0, bitrate=250000)
    # ecu.connect(bustype='vector', app_name='CANalyzer', channel=0, bitrate=250000)
    # ecu.connect(bustype='nican', channel='CAN0', bitrate=250000)    
    # ecu.connect('testchannel_1', bustype='virtual')

    # compose the name descriptor for the new ca
    name = j1939.Name(
        arbitrary_address_capable=0, 
        industry_group=j1939.Name.IndustryGroup.Industrial,
        vehicle_system_instance=1,
        vehicle_system=1,
        function=1,
        function_instance=1,
        ecu_instance=1,
        manufacturer_code=666,
        identity_number=1234567
        )

    # create derived CA with given NAME and ADDRESS
    ca = OwnCaToProduceCyclicMessages(name, 129)
    # add CA to the ECU
    ecu.add_ca(controller_application=ca)
    # by starting the CA it starts the address claiming procedure on the bus
    ca.start()

    time.sleep(120)

    print("Deinitializing")
    ca.stop()
    ecu.disconnect()

if __name__ == '__main__':
    main()        

