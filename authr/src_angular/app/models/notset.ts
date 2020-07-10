import {Color} from './color';
import {Agent, AgentType} from './agent';
import {Thing, ThingType} from './thing';
import {Destination} from './destination';

export function getNotSetAgent(): Agent {
  return {id:"notset",
          name: "Unset Agent",
          timestamp: 0,
          type:AgentType.NOTSET,
          arms:[],
          bgColor:'rgb(244,67,54)',
          txtColor:'white',
          color:{r: 244, g: 67, b: 54},
          useIcon:true
      }
}

export function getOptAgent(): Agent {
  return {id:"OPTIMIZE_DIRECTIVE",
          name: "Optimize",
          timestamp: 0,
          type:AgentType.OPT,
          arms:[],
          bgColor:'rgb(0,0,0)',
          txtColor:'white',
          color:{r: 0, g: 0, b: 0},
          useIcon:true
      }
}

export function getNotSetThing(): Thing {
  return {id:"notset",
          name: "Unset Thing",
          timestamp: 0,
          type:ThingType.NOTSET,
          bgColor:'rgb(244,67,54)',
          color:{r: 244, g: 67, b: 54},
          txtColor:'white',
          size: 1,
          useIcon:true
      }
}

export function getNotSetDestination(): Destination {
  return {id:"notset",
          name: "Unset Destination",
          type: "notset",
          timestamp: 0,
          color:{r: 244, g: 67, b: 54},
          bgColor:'rgb(244,67,54)',
          txtColor:'white',
          useIcon:true,
          position: {x:null,y:null,z:null,},
          orientation: {x:null,y:null,z:null},
          movable: false,
          reachable: false,
          locked: false
      }
}
