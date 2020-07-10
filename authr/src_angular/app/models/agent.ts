import {Color} from './color';

export enum AgentType {
  HUMAN = "human",
  ROBOT = "robot",
  NOTSET = 'notset',
  OPT = 'opt'
}

export interface Agent {
  id:string;
  name:string;
  timestamp:number;
  type:AgentType;
  arms:string[];
  bgColor:string;
  txtColor:string;
  color:Color;
  useIcon:boolean;
}
