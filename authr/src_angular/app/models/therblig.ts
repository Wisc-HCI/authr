
export enum TherbligTypes {

  // Physical
  TRANSPORT_EMPTY = "transport_empty",
  GRASP = "grasp",
  TRANSPORT_LOADED = "transport_loaded",
  RELEASE_LOAD = "release_load",
  HOLD = "hold",
  POSITION = "position",
  REST = "rest",
  PREPOSITION = "preposition",

  // Cognitive
  SEARCH = "search",
  FIND = "find",
  SELECT = "select",
  INSPECT = "inspect",
  PLAN = "plan",

  // Cogntive_Physical
  ASSEMBLE = "assemble",
  DISASSEMBLE = "disassemble",
  USE = "use"
}

export interface Therblig {
  id:string;
  type:TherbligTypes;
  parameters:{[key: string]: any};
  physical:boolean;
  cognitive:boolean;
  allowed:TherbligTypes[];
  sets:{[key:string]: any};
  requires:{[key:string]: any};
  duration:{[key: string]: number};
  cost:{[key: string]: number};
  tooltip:string,
  errors:{[key: string]: string[]};
  showDuration:boolean;
  showCost:boolean;
  settable:string[];
}
