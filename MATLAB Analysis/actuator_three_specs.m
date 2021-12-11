function [torque,ang_vel] = actuator_three_specs(L_l3, m_L3, m_M3)

%determine angular acceleration needs in optimal low need path
%low need path assumes linear increase in ang vel up to ang vel max before
%linearly slowing back down to a stop at the desired end pos.

delta_theta = pi/2; %rad
delta_t = 0.8; %second
ang_vel = delta_theta * 2 / delta_t; %peak ang vel rad/s
ang_accel = (ang_vel - 0) / (delta_t/2); %rad/s^2

%need to find total torque - two parts
%first part is torque to hold load -- assume worst case of links fully
%extended perpendicular to gravity

torque_link_load = m_L3 * 9.81 * (L_l3/2);
%also add torque assuming want to apply 3N end effector force
torque_contact = 3 * L_l3;
%sum two loads...
torque_loads = torque_link_load + torque_contact;


%second part is torque to accelerate the links
%1) torque to accelerate motor, and 2) torque to accelerate links/motors beyond

%we will do 1) first...
I_motor = calc_I(m_M3, .005); %will assume this as overestimate of mass and length of shaft
torque_motor_accel = I_motor * ang_accel;
%now we will do 2) which is dominant load torque...
I_link = calc_I(m_L3, L_l3);
torque_link_accel = I_link * ang_accel;
%sum two acceleration needs...
torque_accel = torque_motor_accel + torque_link_accel;

%sum torque needs from balancing the load and accelerating the load...
torque = torque_loads + torque_accel;

%convert to easy to spec units...
torque = get_oz_in(get_kg_cm(torque)); %oz-in
ang_vel = get_rpm(ang_vel); %rpm

end