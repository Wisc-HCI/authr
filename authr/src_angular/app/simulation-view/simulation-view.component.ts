import { Component, OnInit } from '@angular/core';
import { AuthrService } from '../services/authr.service';
import { Agent } from '../models/agent';
import { KeyValue } from '@angular/common';

@Component({
  selector: 'app-simulation-view',
  templateUrl: './simulation-view.component.html',
  styleUrls: ['./simulation-view.component.css'],
  host: {'class': 'content-flex'}
})
export class SimulationViewComponent {

  public loaded: boolean = false;
  public hasErrors: boolean = false;
  public errors: string[] = [];
  public therbligErrors: {};
  public therbligKeys: string[];
  public data: any;
  public scaling:number = 100;

  private time:number;
  private activeTherbligs:string[] = [];
  private paused:boolean;
  private running:boolean;

  public restartBtnDisabled:boolean;
  public startBtnDisabled:boolean;
  public pauseBtnDisabled:boolean;

  constructor(public authrService:AuthrService) {
    this.authrService.workspace_split=40;
    this.therbligKeys = this.authrService.therbligKeys;
  }

  ngOnInit() {
    this.time = 0;
    this.paused = false;
    this.running = false;

    this.restartBtnDisabled = false;
    this.startBtnDisabled = false;
    this.pauseBtnDisabled = true;

    this.authrService.expandedPlan.subscribe((edata) => {
      this.data = edata;
      this.loaded = true;
      if (edata.verify_error_count > 0 || edata.param_error_count > 0 || !edata.veropt_is_valid) {
        this.hasErrors = true;
        this.errors = [];
        Object.keys(edata.verify_errors).forEach((key)=>{
            Object.keys(edata.verify_errors[key]).forEach((subKey)=>{
                let errorText = edata.verify_errors[key][subKey];
                console.log(errorText);
                if (errorText != null) {
                    // if(this.therbligKeys.indexOf(key) > -1) {
                    //   this.therbligErrors[key] = errorText;
                    // }
                    // else {
                      this.errors.push(errorText);
                    // }
                    console.log(errorText);
                }
            })
        })
        Object.keys(edata.param_errors).forEach((key)=>{
            Object.keys(edata.param_errors[key]).forEach((subKey)=>{
                let errorText = edata.param_errors[key][subKey];
                console.log(errorText);
                if (errorText != null && errorText != {}) {
                    // if(this.therbligKeys.indexOf(key) > -1) {
                    //   this.therbligErrors[key] = errorText;
                    // }
                    // else {
                      this.errors.push(errorText);
                    // }
                    console.log(errorText);
                }
            })
        })
        Object.keys(edata.veropt_errors).forEach((key)=>{
            Object.keys(edata.veropt_errors[key]).forEach((subKey)=>{
                let errorText = edata.veropt_errors[key][subKey];
                if(errorText != "None") {
                    console.log(errorText);
                    if (errorText != null && errorText != {}) {
                        // if(this.therbligKeys.indexOf(key) > -1) {
                        //   this.therbligErrors[key] = errorText;
                        // }
                        // else {
                          this.errors.push(errorText);
                        // }
                        console.log(errorText);
                    }
                }
            })
        })
    } else {
        this.hasErrors = false;
        this.errors = [];
    }
    });

    this.authrService.markerVisibilityTopic.publish({"data":"simple"});
  }

  ngOnDestroy() {
    this.time = 0;
    this.paused = false;

    this.restartBtnDisabled = false;
    this.startBtnDisabled = false;
    this.pauseBtnDisabled = false;

    this.authrService.simulate(this.time, 'reset', (f) => {
      console.log('Restart - Feedback',f);
    },(r) => {
      console.log('Restart - Finished',r);
    });
    this.authrService.markerVisibilityTopic.publish({"data":"interactive"});
  }

  sortAgentByTimestamp = (a: KeyValue<string,Agent>, b: KeyValue<string,Agent>): number => {
      return a.value.timestamp < b.value.timestamp ? -1 : (b.value.timestamp < a.value.timestamp ? 1 : 0);
  }

  getTherblig(therblig): any {
    let returnval = null;
    if (this.data) {
      Object.keys(this.data.expanded_plan).forEach((agent) => {
        this.data.expanded_plan[agent].forEach(therbligObj => {
          // console.log(therbligObj.therblig["eid"],therblig)
          if (therbligObj.therblig["eid"] == therblig) {
            returnval = therbligObj.therblig
          }
        })
      })
    }
    return returnval
  }

  start() {
    this.restartBtnDisabled = false;
    this.startBtnDisabled = true;
    this.pauseBtnDisabled = false;

    this.running = true;
    this.paused = false;
    console.log('start simulation');
    this.authrService.simulate(this.time, 'simulate', (f) => {
      // console.log('Play - Feedback',f);
      if (this.running) {
        this.time = f.elapsed_time;
        this.activeTherbligs=[];
        let atmap = JSON.parse(f.agent_therblig_map);
        Object.keys(atmap).forEach(agent =>{this.activeTherbligs.push(atmap[agent])});
      }

      //console.log(this.activeTherbligs);
      // console.log(this.time);
    },(r) => {
      this.running = false;
      console.log('Play - Finished',r);

      if (r.status == -1) { // error
        this.paused = false;
      }

      if (!this.paused) {
        this.time = 0;

        this.restartBtnDisabled = false;
        this.startBtnDisabled = false;
        this.pauseBtnDisabled = true;
      }

      console.log(this.time);
    });
  }

  restart() {

    this.restartBtnDisabled = false;
    this.startBtnDisabled = true;
    this.pauseBtnDisabled = true;

    this.paused = false;
    this.time = 0;
    console.log("stop simulation");
    this.authrService.simulate(this.time, 'reset', (f) => {
      console.log('Restart - Feedback',f);
    },(r) => {
      console.log('Restart - Finished',r);
      this.time = 0;

      this.restartBtnDisabled = false;
      this.startBtnDisabled = false;
      this.pauseBtnDisabled = true;
    });
  }

  pause() {

    this.restartBtnDisabled = false;
    this.startBtnDisabled = false;
    this.pauseBtnDisabled = true;

    this.paused = true;
    console.log("pause simulation");
    this.authrService.simulate(this.time, 'pause', (f) => {
      console.log('Pause - Feedback',f);
    },(r) => {
      console.log('Pause - Finished',r);
    });
  }

}
