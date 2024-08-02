import numpy as np
import skfuzzy as fuzzy
from skfuzzy import control as ctrl

def initYawFuzzy():
    ## Define the Input Variables and their Membership Functions
    # Yaw Rate Error
    yaw_rate_err =  ctrl.Antecedent(np.arange(-1, 1, 0.01), 'yaw_rate_err')   

    yaw_rate_err['nl'] = fuzzy.trapmf(yaw_rate_err.universe, [-1, -1, -0.75, -0.25])
    yaw_rate_err['ns'] = fuzzy.trimf(yaw_rate_err.universe, [-0.75, -0.25, -0.1])
    yaw_rate_err['ze'] = fuzzy.trapmf(yaw_rate_err.universe, [-0.25, -0.1, 0.1, 0.25])
    yaw_rate_err['ps'] = fuzzy.trimf(yaw_rate_err.universe, [0.1, 0.25, 0.75])
    yaw_rate_err['pl'] = fuzzy.trapmf(yaw_rate_err.universe, [0.25, 0.75, 1, 1])

    # Yaw Rate Error Derivative
    yaw_rate_err_dt = ctrl.Antecedent(np.arange(-20, 20.05, 0.05), 'yaw_rate_err_dt')

    yaw_rate_err_dt['nl'] = fuzzy.trapmf(yaw_rate_err_dt.universe, [-20, -20, -18, -8])
    yaw_rate_err_dt['ns'] = fuzzy.trimf(yaw_rate_err_dt.universe, [-18, -8, -3])
    yaw_rate_err_dt['ze'] = fuzzy.trapmf(yaw_rate_err_dt.universe, [-8, -3, 3, 8])
    yaw_rate_err_dt['ps'] = fuzzy.trimf(yaw_rate_err_dt.universe, [3, 8, 18])
    yaw_rate_err_dt['pl'] = fuzzy.trapmf(yaw_rate_err_dt.universe, [8, 18, 20, 20])


    # Sideslip Angle Error
    sideslip_err = ctrl.Antecedent(np.arange(-0.5, 0.51, 0.1), 'sideslip_err')

    sideslip_err['n'] = fuzzy.trapmf(sideslip_err.universe, [-0.5, -0.5, -0.4, -0.1])
    sideslip_err['z'] = fuzzy.trapmf(sideslip_err.universe, [-0.4, -0.1, 0.1, 0.4])
    sideslip_err['p'] = fuzzy.trapmf(sideslip_err.universe, [0.1, 0.4, 0.5, 0.5])


    ## Define the Range and Membership Functions for the Output Variable: Torque Correction
    T_YS_correction = ctrl.Consequent(np.arange(-1, 1.001, 0.001), 'T_YS_correction')

    T_YS_correction['nvl'] = fuzzy.trapmf(T_YS_correction.universe, [-1, -1, -0.8, -0.6])
    T_YS_correction['nl'] = fuzzy.trimf(T_YS_correction.universe, [-0.8, -0.6, -0.4])
    T_YS_correction['nm'] = fuzzy.trimf(T_YS_correction.universe, [-0.6, -0.4, -0.2])
    T_YS_correction['ns'] = fuzzy.trimf(T_YS_correction.universe, [-0.4, -0.2, 0])
    T_YS_correction['ze'] = fuzzy.trimf(T_YS_correction.universe, [-0.2, 0, 0.2])
    T_YS_correction['ps'] = fuzzy.trimf(T_YS_correction.universe, [0, 0.2, 0.4])
    T_YS_correction['pm'] = fuzzy.trimf(T_YS_correction.universe, [0.2, 0.4, 0.6])
    T_YS_correction['pl'] = fuzzy.trimf(T_YS_correction.universe, [0.4, 0.6, 0.8])
    T_YS_correction['pvl'] = fuzzy.trapmf(T_YS_correction.universe, [0.6, 0.8, 1, 1])


    ## Define the Fuzzy Rules
    # Sideslip Angle Error is Negative
    rule1 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['nl'] & sideslip_err['n'], T_YS_correction['nvl'])
    rule2 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ns'] & sideslip_err['n'], T_YS_correction['nvl'])
    rule3 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ze'] & sideslip_err['n'], T_YS_correction['nl'])
    rule4 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ps'] & sideslip_err['n'], T_YS_correction['nl'])
    rule5 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['pl'] & sideslip_err['n'], T_YS_correction['nm'])

    rule6 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['nl'] & sideslip_err['n'], T_YS_correction['nvl'])
    rule7 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ns'] & sideslip_err['n'], T_YS_correction['nl'])
    rule8 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ze'] & sideslip_err['n'], T_YS_correction['nm'])
    rule9 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ps'] & sideslip_err['n'], T_YS_correction['ns'])
    rule10 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['pl'] & sideslip_err['n'], T_YS_correction['ze'])

    rule11 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['nl'] & sideslip_err['n'], T_YS_correction['nm'])
    rule12 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ns'] & sideslip_err['n'], T_YS_correction['ns'])
    rule13 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ze'] & sideslip_err['n'], T_YS_correction['ze'])
    rule14 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ps'] & sideslip_err['n'], T_YS_correction['ps'])
    rule15 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['pl'] & sideslip_err['n'], T_YS_correction['pm'])

    rule16 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['nl'] & sideslip_err['n'], T_YS_correction['ze'])
    rule17 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ns'] & sideslip_err['n'], T_YS_correction['ps'])
    rule18 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ze'] & sideslip_err['n'], T_YS_correction['pm'])
    rule19 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ps'] & sideslip_err['n'], T_YS_correction['pl'])
    rule20 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['pl'] & sideslip_err['n'], T_YS_correction['pvl'])

    rule21 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['nl'] & sideslip_err['n'], T_YS_correction['pm'])
    rule22 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ns'] & sideslip_err['n'], T_YS_correction['pl'])
    rule23 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ze'] & sideslip_err['n'], T_YS_correction['pl'])
    rule24 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ps'] & sideslip_err['n'], T_YS_correction['pvl'])
    rule25 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['pl'] & sideslip_err['n'], T_YS_correction['pvl'])

    # Sideslip Angle Error is Zero
    rule26 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['nl'] & sideslip_err['z'], T_YS_correction['nvl'])
    rule27 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ns'] & sideslip_err['z'], T_YS_correction['nvl'])
    rule28 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ze'] & sideslip_err['z'], T_YS_correction['nl'])
    rule29 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ps'] & sideslip_err['z'], T_YS_correction['nl'])
    rule30 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['pl'] & sideslip_err['z'], T_YS_correction['nm'])

    rule31 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['nl'] & sideslip_err['z'], T_YS_correction['nvl'])
    rule32 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ns'] & sideslip_err['z'], T_YS_correction['nl'])
    rule33 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ze'] & sideslip_err['z'], T_YS_correction['nm'])
    rule34 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ps'] & sideslip_err['z'], T_YS_correction['ns'])
    rule35 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['pl'] & sideslip_err['z'], T_YS_correction['ze'])

    rule36 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['nl'] & sideslip_err['z'], T_YS_correction['nm'])
    rule37 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ns'] & sideslip_err['z'], T_YS_correction['ns'])
    rule38 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ze'] & sideslip_err['z'], T_YS_correction['ze'])
    rule39 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ps'] & sideslip_err['z'], T_YS_correction['ps'])
    rule40 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['pl'] & sideslip_err['z'], T_YS_correction['pm'])

    rule41 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['nl'] & sideslip_err['z'], T_YS_correction['ze'])
    rule42 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ns'] & sideslip_err['z'], T_YS_correction['ps'])
    rule43 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ze'] & sideslip_err['z'], T_YS_correction['pm'])
    rule44 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ps'] & sideslip_err['z'], T_YS_correction['pl'])
    rule45 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['pl'] & sideslip_err['z'], T_YS_correction['pvl'])

    rule46 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['nl'] & sideslip_err['z'], T_YS_correction['pm'])
    rule47 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ns'] & sideslip_err['z'], T_YS_correction['pl'])
    rule48 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ze'] & sideslip_err['z'], T_YS_correction['pl'])
    rule49 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ps'] & sideslip_err['z'], T_YS_correction['pvl'])
    rule50 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['pl'] & sideslip_err['z'], T_YS_correction['pvl'])

    # Sideslip Angle Error is Positive
    rule51 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['nl'] & sideslip_err['p'], T_YS_correction['nvl'])
    rule52 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ns'] & sideslip_err['p'], T_YS_correction['nvl'])
    rule53 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ze'] & sideslip_err['p'], T_YS_correction['nl'])
    rule54 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['ps'] & sideslip_err['p'], T_YS_correction['nl'])
    rule55 = ctrl.Rule(yaw_rate_err['nl'] & yaw_rate_err_dt['pl'] & sideslip_err['p'], T_YS_correction['nm'])

    rule56 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['nl'] & sideslip_err['p'], T_YS_correction['nvl'])
    rule57 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ns'] & sideslip_err['p'], T_YS_correction['nl'])
    rule58 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ze'] & sideslip_err['p'], T_YS_correction['nm'])
    rule59 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['ps'] & sideslip_err['p'], T_YS_correction['ns'])
    rule60 = ctrl.Rule(yaw_rate_err['ns'] & yaw_rate_err_dt['pl'] & sideslip_err['p'], T_YS_correction['ze'])

    rule61 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['nl'] & sideslip_err['p'], T_YS_correction['nm'])
    rule62 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ns'] & sideslip_err['p'], T_YS_correction['ns'])
    rule63 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ze'] & sideslip_err['p'], T_YS_correction['ze'])
    rule64 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['ps'] & sideslip_err['p'], T_YS_correction['ps'])
    rule65 = ctrl.Rule(yaw_rate_err['ze'] & yaw_rate_err_dt['pl'] & sideslip_err['p'], T_YS_correction['pm'])

    rule66 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['nl'] & sideslip_err['p'], T_YS_correction['ze'])
    rule67 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ns'] & sideslip_err['p'], T_YS_correction['ps'])
    rule68 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ze'] & sideslip_err['p'], T_YS_correction['pm'])
    rule69 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['ps'] & sideslip_err['p'], T_YS_correction['pl'])
    rule70 = ctrl.Rule(yaw_rate_err['ps'] & yaw_rate_err_dt['pl'] & sideslip_err['p'], T_YS_correction['pvl'])

    rule71 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['nl'] & sideslip_err['p'], T_YS_correction['pm'])
    rule72 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ns'] & sideslip_err['p'], T_YS_correction['pl'])
    rule73 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ze'] & sideslip_err['p'], T_YS_correction['pl'])
    rule74 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['ps'] & sideslip_err['p'], T_YS_correction['pvl'])
    rule75 = ctrl.Rule(yaw_rate_err['pl'] & yaw_rate_err_dt['pl'] & sideslip_err['p'], T_YS_correction['pvl'])


    ## Create the Controller
    YS_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                            rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                            rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30,
                            rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40,
                            rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50,
                            rule51, rule52, rule53, rule54, rule55, rule56, rule57, rule58, rule59, rule60,
                            rule61, rule62, rule63, rule64, rule65, rule66, rule67, rule68, rule69, rule70,
                            rule71, rule72, rule73, rule74, rule75])

    YS_controller = ctrl.ControlSystemSimulation(YS_system)

    return YS_controller


def yawCorrection(YS_controller, yaw_rate_err, yaw_rate_err_dt, sideslip_err):
    # Controller Inputs
    YS_controller.input['yaw_rate_err'] = yaw_rate_err
    YS_controller.input['yaw_rate_err_dt'] = yaw_rate_err_dt
    YS_controller.input['sideslip_err'] = sideslip_err

    YS_controller.compute()

    # Controller Output
    yaw_correction = YS_controller.output['T_YS_correction']

    return yaw_correction
