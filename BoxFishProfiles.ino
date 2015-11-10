//
// Title: BoxFishOven Profiles
// Author: Orange Cat
// Date: 2015-11-11
//
// This sketch contains all the code necessary to add new profiles to BoxFishOven.
//
// To add a new profile:
//    Add a new BoxFishMenuItem which describes the profile,
//    Add new menus to buildMenus, using the BoxFishMenuItem you added for the new profile and add() them into the hierarchy.
//    In menuItemWasSelected, call your profile setup when item is your new BoxFishMenuItem.
// 
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//

enum BoxFishMenuItem {
  // start numbering at 1, because 0 indicates no menu item
  kBoxFishMenuItemLeaded = 1,
  kBoxFishMenuItemLeadFree,
  
  kBoxFishMenuItemSetThickness,
        
  kBoxFishMenuItemAcrylic,
  kBoxFishMenuItemPolycarbonate,
  kBoxFishMenuItemAcetal,

  kBoxFishMenuItemCureEpoxy80,

  kBoxFishMenuItemDesiccantSilicaGel,
  
  kBoxFishMenuItemRapidCool,
  kBoxFishMenuItemReset
};

void buildMenus()
{
  MenuItemRef root = ui.getRootMenu();

  // create our menu system
  static MenuItem mi_reflow = MenuItem(F("[Reflow]"));
  static MenuItem mi_reflow_lead = MenuItem(F("[Leaded]"));
  static MenuItem mi_reflow_lead_use = MenuItem(F("Run Leaded"), kBoxFishMenuItemLeaded);
  static MenuItem mi_reflow_lead_free = MenuItem(F("[Lead Free]"));
  static MenuItem mi_reflow_lead_free_use = MenuItem(F("Run Lead Free"), kBoxFishMenuItemLeadFree);

  static MenuItem mi_anneal = MenuItem(F("[Anneal]"));
  static MenuItem mi_anneal_thickness = MenuItem(F("[Set Thickness]"), kBoxFishMenuItemSetThickness);
  static MenuItem mi_anneal_acrylic = MenuItem(F("[Acrylic]"));
  static MenuItem mi_anneal_acrylic_use = MenuItem(F("Run Acrylic"), kBoxFishMenuItemAcrylic);
  static MenuItem mi_anneal_polycarbonate = MenuItem(F("[Polycarbonate]"));
  static MenuItem mi_anneal_polycarbonate_use = MenuItem(F("Run Polycarbonate"), kBoxFishMenuItemPolycarbonate);
  static MenuItem mi_anneal_acetal = MenuItem(F("[Acetal]"));
  static MenuItem mi_anneal_acetal_use = MenuItem(F("Run Acetal"), kBoxFishMenuItemAcetal);

  static MenuItem mi_cure = MenuItem(F("[Cure]"));
  static MenuItem mi_cure_epoxy80 = MenuItem(F("[Epoxy Staged 80]"));
  static MenuItem mi_cure_epoxy80_use = MenuItem(F("Run Epoxy Staged 80"), kBoxFishMenuItemCureEpoxy80);
  
  static MenuItem mi_desiccant = MenuItem(F("[Desiccant]"));
  static MenuItem mi_desiccant_silica_gel = MenuItem(F("[Silica Gel]"));
  static MenuItem mi_desiccant_silica_gel_use = MenuItem(F("Run Silica Gel"), kBoxFishMenuItemDesiccantSilicaGel);
    
  static MenuItem mi_rapid = MenuItem(F("[Rapid Cool]"));
  static MenuItem mi_rapid_use = MenuItem(F("Run Rapid Cool"), kBoxFishMenuItemRapidCool);

  static MenuItem mi_system = MenuItem(F("[System]"));
  static MenuItem mi_system_version = MenuItem(F("[Version]"));
  static MenuItem mi_system_version_show = MenuItem(kBoxFishOvenVersion);
  static MenuItem mi_system_reset = MenuItem(F("[Reset]"));
  static MenuItem mi_system_reset_use = MenuItem(F("Reset Controller"), kBoxFishMenuItemReset);

  // setup the menu hierarchy
  root.add(mi_reflow).add(mi_anneal).add(mi_cure).add(mi_desiccant).add(mi_rapid).add(mi_system);

  // configure the remaining menus
  mi_reflow.addRight(mi_reflow_lead).add(mi_reflow_lead_free);
  mi_reflow_lead.addRight(mi_reflow_lead_use);
  mi_reflow_lead_free.addRight(mi_reflow_lead_free_use);

  mi_anneal.addRight(mi_anneal_thickness).add(mi_anneal_acrylic).add(mi_anneal_polycarbonate).add(mi_anneal_acetal);
  mi_anneal_acrylic.addRight(mi_anneal_acrylic_use);
  mi_anneal_polycarbonate.addRight(mi_anneal_polycarbonate_use);
  mi_anneal_acetal.addRight(mi_anneal_acetal_use);

  mi_cure.addRight(mi_cure_epoxy80);
  mi_cure_epoxy80.addRight(mi_cure_epoxy80_use);

  mi_desiccant.addRight(mi_desiccant_silica_gel);
  mi_desiccant_silica_gel.addRight(mi_desiccant_silica_gel_use);
  
  mi_rapid.addRight(mi_rapid_use);

  mi_system.addRight(mi_system_version).add(mi_system_reset);
  mi_system_reset.addRight(mi_system_reset_use);
  mi_system_version.addRight(mi_system_version_show);
}

void menuItemWasSelected(int item)
{
  // this function is called when the user selects one of the menu items where a second
  // argument was passed to MenuItem during creation. The name of this function was
  // passed to ui.begin() in setup().

  unsigned long hold_time_min = (mmThickness/6.35) * 30.0 + 0.5;
  
  switch (item) {
    case kBoxFishMenuItemLeadFree:
      // standard lead free profile, peak 245C
      startReflow(245.0);
      break;

    case kBoxFishMenuItemLeaded:
      // standard leaded profile, peak 225C
      startReflow(225.0);
      break;

    case kBoxFishMenuItemSetThickness:
      // allows user to set the hold_time for annealing by selecting the material thickness
      menuSetThickness();
      break;

    case kBoxFishMenuItemAcrylic:
      // ramp up to 82C over 2 hrs, hold for hold_time_min minutes, and ramp down to 30C at 27.8C/hr
      startAnneal(82.0, 2*60, hold_time_min, 27.8);
      break;

    case kBoxFishMenuItemPolycarbonate:
      // ramp up to 135C over 4 hrs, hold for hold_time_min minutes, and ramp down to 30C at 27.8C/hr
      startAnneal(135.0, 4*60, hold_time_min, 27.8);
      break;
      
    case kBoxFishMenuItemAcetal:
      // ramp up to 160C over 4 hrs, hold for hold_time_min minutes, and ramp down to 30C at 27.8C/hr
      startAnneal(160.0, 4*60, hold_time_min, 27.8);
      break;

    case kBoxFishMenuItemCureEpoxy80:
      // ramp up slowly to 40C, hold for 60 minutes, ramp up slowly to 80C and hold for 180 minutes, then slow cool
      startEpoxy(40.0, 60.0, 80.0, 180.0);
      break;

    case kBoxFishMenuItemDesiccantSilicaGel:
      // dry at 130C for 2 hrs (complete when cool at 50C)
      startDry(130.0, 2*60);
      break;

    case kBoxFishMenuItemRapidCool:
      startRapidCool();
      break;

    case kBoxFishMenuItemReset:
      reset();
      break;

    default:
      // do nothing
      break;
  }
}

// reusable PID operation objects to save memory
PIDOp preheat;
PIDOp soak;
PIDOp reflow;
PIDOp cool;

void startReflow(double reflow_temperature)
{
  // For profile see:
  //  http://www.kaschke.de/fileadmin/user_upload/documents/datenblaetter/Induktivitaeten/Reflowprofile.pdf
  //  http://www.compuphase.com/electronics/reflowsolderprofiles.htm

  // ready for a new job
  ovenBegin();
  setSerialLogSeconds(1);

  // preheat: raise temperature quickly to 150C
  preheat.begin(150.0, 112.0, 0.02, 252.0);
  preheat.setEpsilon(4.0);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Preheat"));
  ovenSeq.addOp(preheat);
  
  // soak: now raise temperature to 200C over 100 seconds
  soak.begin(200.0, 622.0, 0.22, 1122.0);
  soak.setRampTime(100);
  soak.setEpsilon(4.0);
  soak.setControlLimits(0.0, kWindowSize);
  soak.setName(F("Soak"));
  ovenSeq.addOp(soak);
  
  // reflow: raise temperature quickly to the reflow_temperature
  reflow.begin(reflow_temperature+4.0, 242.0, 0.0, 25.0);
  reflow.setHoldTime(10); // hold for 10 seconds to allow pins on metal connectors to reflow
  reflow.setEpsilon(5.0);
  reflow.setControlLimits(0.0, kWindowSize);
  reflow.setName(F("Reflow"));
  ovenSeq.addOp(reflow);
  
  // cool: cool quickly using the blower
  cool.begin(50.0, 40.0, 0.01, 30.0);
  cool.setReverse(true);
  cool.setEpsilon(3.0);
  cool.setControlLimits(0.0, kBlowerPWMMax);
  cool.setName(F("Cool"));
  ovenSeq.addOp(cool);

  // start the sequence
  ovenRun();
}

void startAnneal(double hold_temp, unsigned long ramp_up_minutes, unsigned long hold_minutes, double ramp_down_deg_hour)
{
  // All temps in C. For profiles (must convert F to C) see:
  //  http://www.boedeker.com/anneal.htm
  
  // ready for a new job
  ovenBegin();
  setSerialLogSeconds(60);
  
  double epsilon = 3.0;

  // slowly increase heat from room temperature to hold_temp (C) over ramp_up_minutes minnutes
  preheat.begin(hold_temp, 300.0, 0.03, 200.0);
  preheat.setRampTime(ramp_up_minutes * 60uL);
  preheat.setEpsilon(epsilon);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Slow Heat"));
  ovenSeq.addOp(preheat);

  // hold at hold_temp for hold_minutes minutes
  soak.begin(hold_temp, 300.0, 0.02, 200.0);
  soak.setHoldTime(hold_minutes * 60uL);
  soak.setEpsilon(epsilon);
  soak.setControlLimits(0.0, kWindowSize);
  soak.setName(F("Hold"));
  ovenSeq.addOp(soak);
  
  // cool to 40C at ramp_down_deg_hour degrees C/hr (note this is actually a heating cycle with decreasing setpoint)
  unsigned long ramp_down_minutes = ((hold_temp - 40.0) * 60.0) / ramp_down_deg_hour + 0.5;
  cool.begin(40.0, 300.0, 0.03, 200.0);
  cool.setRampTime(ramp_down_minutes * 60uL);
  cool.setEpsilon(epsilon);
  cool.setControlLimits(0.0, kWindowSize);
  cool.setName(F("Slow Cool"));
  ovenSeq.addOp(cool);

  // cool actively to 30C at ramp_down_deg_hour degrees C/hr
  ramp_down_minutes = ((40.0 - 30.0) * 60.0) / ramp_down_deg_hour + 0.5;
  reflow.begin(30.0, 38, 0.008, 32);
  reflow.setRampTime(ramp_down_minutes * 60uL);
  reflow.setEpsilon(epsilon);
  reflow.setReverse(true);
  reflow.setControlLimits(0.0, kBlowerPWMMax);
  reflow.setName(F("Active Cool"));
  ovenSeq.addOp(reflow);

  // start the sequence
  ovenRun();
}

void startEpoxy(double hold_temp1, unsigned long hold_minutes1, unsigned long hold_temp2, unsigned long hold_minutes2)
{
  // ready for a new job
  ovenBegin();
  setSerialLogSeconds(60);
  
  double epsilon = 3.0;

  // slowly increase heat from room temperature to hold_temp1 (C) over 20 minutes then hold for hold_minutes1
  preheat.begin(hold_temp1, 300.0, 0.03, 200.0);
  preheat.setRampTime(20uL * 60uL);
  preheat.setHoldTime(hold_minutes1 * 60uL);
  preheat.setEpsilon(epsilon);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Cure Temp1"));
  ovenSeq.addOp(preheat);

  // slowly increase heat from hold_temp1 to hold_temp2 (C) over 20 minutes then hold for hold_minutes2
  soak.begin(hold_temp2, 300.0, 0.03, 200.0);
  soak.setRampTime(20uL * 60uL);
  soak.setHoldTime(hold_minutes2 * 60uL);
  soak.setEpsilon(epsilon);
  soak.setControlLimits(0.0, kWindowSize);
  soak.setName(F("Cure Temp2"));
  ovenSeq.addOp(soak);
  
  // cool to 30C at 50C/hr (note this is actually a heating cycle with decreasing setpoint)
  double ramp_down_deg_hour = 50.0;
  unsigned long ramp_down_minutes = ((hold_temp2 - 30.0) * 60.0) / ramp_down_deg_hour + 0.5;
  cool.begin(30.0, 300.0, 0.03, 200.0);
  cool.setRampTime(ramp_down_minutes * 60uL);
  cool.setEpsilon(epsilon);
  cool.setControlLimits(0.0, kWindowSize);
  cool.setName(F("Cool Down"));
  ovenSeq.addOp(cool);

  // start the sequence
  ovenRun();
}

void startDry(double dry_temp, unsigned long hold_minutes)
{
  // ready for a new job
  ovenBegin();
  setSerialLogSeconds(60);
  
  double epsilon = 3.0;

  // take temp up to dry temp and hold for hold_minutes
  preheat.begin(dry_temp, 300.0, 0.03, 200.0);
  preheat.setHoldTime(hold_minutes * 60uL);
  preheat.setEpsilon(epsilon);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Drying"));
  ovenSeq.addOp(preheat);
  
  // cool to 50C
  cool.begin(50.0, 38, 0.008, 32);
  cool.setEpsilon(epsilon);
  cool.setReverse(true);
  cool.setControlLimits(0.0, kBlowerPWMMax);
  cool.setName(F("Cool"));
  ovenSeq.addOp(cool);

  // start the sequence
  ovenRun();
}


void startRapidCool()
{
  // ready for a new job
  ovenBegin();
  setSerialLogSeconds(1);

  // aggressively cool to 40.0C
  cool.begin(40.0 - 5.0, 38, 0.008, 32);
  cool.setEpsilon(5.0);
  cool.setReverse(true);
  cool.setControlLimits(0.0, kBlowerPWMMax);
  cool.setName(F("Rapid Cool"));
  ovenSeq.addOp(cool);
  
  // start the sequence
  ovenRun();
}

