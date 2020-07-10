import { Component, Input, OnInit, OnDestroy } from '@angular/core';
import { AuthrService }                        from "../services/authr.service";
import { Agent, AgentType }                    from '../models/agent';
import { enumSelector, hexToRgb, rgbToHex }    from "../utility.module";
import { MatSnackBar }                         from '@angular/material';

@Component({
  selector: 'agent',
  templateUrl: './element-agent.component.html',
  styleUrls: ['./element-agent.component.css']
})
export class ElementAgentComponent {

  @Input() key: string;
  public agentTypes = enumSelector(AgentType);
  public colorUpdateFunction: (value:any) => void;

  private icon = "edit";
  private tip = "Edit Mode";
  private showEdit = false;
  private prevName = "";
  private prevType = "";
  private prevColor = "";
  private agentName = "";
  private agentType = "";
  private emptyFlag = 1;

  constructor(public snackbar: MatSnackBar,
              public authrService:AuthrService) {}

  public get name(): string {
    if (this.emptyFlag === 1) {
      this.agentName = this.authrService.getAgentById(this.key).name;
      this.agentType = this.authrService.getAgentById(this.key).type;
      this.emptyFlag = 0;
    }
    return this.agentName;
  }

  public set name(event) {
    this.agentName = event;
  }

  public get type(): string {
    if (this.emptyFlag === 1) {
      this.agentName = this.authrService.getAgentById(this.key).name;
      this.agentType = this.authrService.getAgentById(this.key).type;
      this.emptyFlag = 0;
    }
    return this.agentType;
  }

  public set type(event) {
    this.agentType = event;
  }

  public set color(event) {
    this.authrService.setAgentById({color:hexToRgb(event),elementSetType:'agent'},this.key);
  }

  public get color(): string {
      return rgbToHex(
        this.authrService.agents[this.key].color.r,
        this.authrService.agents[this.key].color.g,
        this.authrService.agents[this.key].color.b
      );
  }

  public emitCustomError(message:string) {
    this.snackbar.open(message,null, {duration: 2000});
  }

  switchMode(flag) {
    if (this.icon === "edit") {
      this.icon = "save";
      this.tip = "Save Change";
      this.showEdit = true;
      this.prevName = this.authrService.getAgentById(this.key).name;
      this.prevType = this.authrService.getAgentById(this.key).type;
      this.prevColor = this.authrService.getAgentById(this.key).color;
    } else if (this.icon === "save") {
      if (this.agentName === "") {
        this.emitCustomError("The agent name cannot be empty, please try again.");
        this.emptyFlag = 1;
        return;
      }
      this.icon = "edit";
      this.tip = "Edit Mode";
      this.showEdit = false;
      if (flag === 1) {
        this.agentName = this.prevName;
        this.agentType = this.prevType;
        this.authrService.setAgentById({color:this.prevColor,elementSetType:'agent'},this.key);
        return;
      }
      this.authrService.setAgentById({name:this.agentName,elementSetType:'agent'},this.key);
      this.authrService.setAgentById({type:this.agentType,elementSetType:'agent'},this.key);
    }
  }
}
