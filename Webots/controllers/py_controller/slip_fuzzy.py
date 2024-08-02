import numpy as np
import skfuzzy as fuzzy
from skfuzzy import control as ctrl


def initSlipFuzzy():
    ## Define the Input Variables and their Membership Functions
    # Slip Ratio Error
    slip_ratio_err =  ctrl.Antecedent(np.arange(-10, 10.0001, 0.0001), 'slip_ratio_err')   

    slip_ratio_err['nl'] = fuzzy.trapmf(slip_ratio_err.universe, [-10, -10, -0.02, -0.005])
    slip_ratio_err['nm'] = fuzzy.trimf(slip_ratio_err.universe, [-0.02, -0.005, -0.0001])
    slip_ratio_err['ns'] = fuzzy.trimf(slip_ratio_err.universe, [-0.005, -0.0001, 0])
    slip_ratio_err['ze'] = fuzzy.trimf(slip_ratio_err.universe, [-0.0005, 0, 0.0005])
    slip_ratio_err['ps'] = fuzzy.trimf(slip_ratio_err.universe, [0, 0.0001, 0.005])
    slip_ratio_err['pm'] = fuzzy.trimf(slip_ratio_err.universe, [0.0001, 0.005, 0.02])
    slip_ratio_err['pl'] = fuzzy.trapmf(slip_ratio_err.universe, [0.005, 0.02, 10, 10])

    # Slip Ratio Error Derivative
    slip_ratio_err_dt = ctrl.Antecedent(np.arange(-1000, 1000.05, 0.05), 'slip_ratio_err_dt')

    slip_ratio_err_dt['nl'] = fuzzy.trapmf(slip_ratio_err_dt.universe, [-1000, -1000, -0.5, -0.1])
    slip_ratio_err_dt['nm'] = fuzzy.trimf(slip_ratio_err_dt.universe, [-0.5, -0.1, -0.05])
    slip_ratio_err_dt['ns'] = fuzzy.trimf(slip_ratio_err_dt.universe, [-0.1, -0.05, 0])
    slip_ratio_err_dt['ze'] = fuzzy.trimf(slip_ratio_err_dt.universe, [-0.05, 0, 0.05])
    slip_ratio_err_dt['ps'] = fuzzy.trimf(slip_ratio_err_dt.universe, [0, 0.05, 0.1])
    slip_ratio_err_dt['pm'] = fuzzy.trimf(slip_ratio_err_dt.universe, [0.05, 0.1, 0.5])
    slip_ratio_err_dt['pl'] = fuzzy.trapmf(slip_ratio_err_dt.universe, [0.1, 0.5, 1000, 1000])


    ## Define the Range and Membership Functions for the Output Variable: Torque Correction
    T_SR_correction = ctrl.Consequent(np.arange(-1, 1.001, 0.001), 'T_SR_correction')

    T_SR_correction['nl'] = fuzzy.trapmf(T_SR_correction.universe, [-1, -1, -0.7, -0.5])
    T_SR_correction['nm'] = fuzzy.trimf(T_SR_correction.universe, [-0.7, -0.5, -0.3])
    T_SR_correction['ns'] = fuzzy.trimf(T_SR_correction.universe, [-0.5, -0.3, 0])
    T_SR_correction['ze'] = fuzzy.trimf(T_SR_correction.universe, [-0.3, 0, 0.3])
    T_SR_correction['ps'] = fuzzy.trimf(T_SR_correction.universe, [0, 0.3, 0.5])
    T_SR_correction['pm'] = fuzzy.trimf(T_SR_correction.universe, [0.3, 0.5, 0.7])
    T_SR_correction['pl'] = fuzzy.trapmf(T_SR_correction.universe, [0.5, 0.7, 1, 1])


    ## Define the Fuzzy Rules
    # Slip Ratio Error is Negative Large
    rule1 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['nl'], T_SR_correction['nl'])
    rule2 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['nm'], T_SR_correction['nl'])
    rule3 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['ns'], T_SR_correction['nl'])
    rule4 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['ze'], T_SR_correction['nm'])
    rule5 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['pl'], T_SR_correction['nm'])
    rule6 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['pm'], T_SR_correction['nm'])
    rule7 = ctrl.Rule(slip_ratio_err['nl'] & slip_ratio_err_dt['ps'], T_SR_correction['ns'])

    # Slip Ratio Error is Negative Medium
    rule8 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['nl'], T_SR_correction['nl'])
    rule9 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['nm'], T_SR_correction['nl'])
    rule10 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['ns'], T_SR_correction['nm'])
    rule11 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['ze'], T_SR_correction['nm'])
    rule12 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['pl'], T_SR_correction['nm'])
    rule13 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['pm'], T_SR_correction['ns'])
    rule14 = ctrl.Rule(slip_ratio_err['nm'] & slip_ratio_err_dt['ps'], T_SR_correction['ns'])

    # Slip Ratio Error is Negative Small
    rule15 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['nl'], T_SR_correction['nl'])
    rule16 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['nm'], T_SR_correction['nm'])
    rule17 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['ns'], T_SR_correction['nm'])
    rule18 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['ze'], T_SR_correction['ns'])
    rule19 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['pl'], T_SR_correction['ze'])
    rule20 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['pm'], T_SR_correction['ps'])
    rule21 = ctrl.Rule(slip_ratio_err['ns'] & slip_ratio_err_dt['ps'], T_SR_correction['ps'])

    # Slip Ratio Error is Zero
    rule22 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['nl'], T_SR_correction['nm'])
    rule23 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['nm'], T_SR_correction['ns'])
    rule24 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['ns'], T_SR_correction['ns'])
    rule25 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['ze'], T_SR_correction['ze'])
    rule26 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['pl'], T_SR_correction['ps'])
    rule27 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['pm'], T_SR_correction['ps'])
    rule28 = ctrl.Rule(slip_ratio_err['ze'] & slip_ratio_err_dt['ps'], T_SR_correction['pm'])

    # Slip Ratio Error is Positive Small
    rule29 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['nl'], T_SR_correction['ns'])
    rule30 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['nm'], T_SR_correction['ns'])
    rule31 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['ns'], T_SR_correction['ze'])
    rule32 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['ze'], T_SR_correction['ps'])
    rule33 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['pl'], T_SR_correction['ps'])
    rule34 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['pm'], T_SR_correction['ps'])
    rule35 = ctrl.Rule(slip_ratio_err['ps'] & slip_ratio_err_dt['ps'], T_SR_correction['pm'])

    # Slip Ratio Error is Positive Medium
    rule36 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['nl'], T_SR_correction['ps'])
    rule37 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['nm'], T_SR_correction['pm'])
    rule38 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['ns'], T_SR_correction['pm'])
    rule39 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['ze'], T_SR_correction['pm'])
    rule40 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['pl'], T_SR_correction['pm'])
    rule41 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['pm'], T_SR_correction['pl'])
    rule42 = ctrl.Rule(slip_ratio_err['pm'] & slip_ratio_err_dt['ps'], T_SR_correction['pl'])

    # Slip Ratio Error is Positive Large
    rule43 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['nl'], T_SR_correction['ps'])
    rule44 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['nm'], T_SR_correction['pm'])
    rule45 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['ns'], T_SR_correction['pm'])
    rule46 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['ze'], T_SR_correction['pm'])
    rule47 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['pl'], T_SR_correction['pl'])
    rule48 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['pm'], T_SR_correction['pl'])
    rule49 = ctrl.Rule(slip_ratio_err['pl'] & slip_ratio_err_dt['ps'], T_SR_correction['pl'])


    ## Create the Controller
    SR_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                                rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                                rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30,
                                rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40,
                                rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49])

    SR_controller = ctrl.ControlSystemSimulation(SR_system)

    return SR_controller


def slipCorrection(SR_controller, slip_ratio_err, slip_ratio_err_dt):
    # Controller Inputs
    SR_controller.input['slip_ratio_err'] = slip_ratio_err
    SR_controller.input['slip_ratio_err_dt'] = slip_ratio_err_dt

    SR_controller.compute()

    # Controller Output
    slip_correction = SR_controller.output['T_SR_correction']
    
    return slip_correction