import { Component, OnInit } from '@angular/core';
import { ActivatedRoute, Router } from '@angular/router';
import { AuthrService } from '../services/authr.service';
import { Therblig } from '../models/therblig';
import { MatSnackBar } from '@angular/material';
import { Agent } from '../models/agent';
import { Thing } from '../models/thing';
import { Destination } from '../models/destination';
import { getNotSetAgent, getOptAgent, getNotSetThing, getNotSetDestination } from '../models/notset';

@Component({
  selector: 'app-therblig-detail',
  templateUrl: './therblig-detail.component.html',
  styleUrls: ['./therblig-detail.component.scss'],
  host: {'class': 'content-flex'}
})
export class TherbligDetailComponent implements OnInit {

  public therblig: Therblig;
  public therbligKey: string;
  public panelOpenState = false;

  private optAgent:Agent = getOptAgent();
  private notsetAgent:Agent = getNotSetAgent();
  private notsetThing:Thing = getNotSetThing();
  private notsetDestination:Destination = getNotSetDestination();

  constructor(public route: ActivatedRoute,
              public router: Router,
              public authrService: AuthrService,
              public snackbar: MatSnackBar) {
  }

  ngOnInit() {
    this.route.paramMap.subscribe(params => {
        this.therbligKey = params.get('therblig_id');
        this.therblig = this.authrService.getTherbligById(this.therbligKey);
    });
  }

  private setAgent(key) {
    this.therblig.parameters.agent = key;
    let settings = {...this.therblig.parameters};
    this.authrService.setTherbligById({"parameters":settings},this.therbligKey);
  }

  private setThing(key) {
    this.therblig.parameters.thing = key;
    let settings = {...this.therblig.parameters};
    this.authrService.setTherbligById({"parameters":settings},this.therbligKey);
  }

  private setDestination(key) {
    this.therblig.parameters.destination = key;
    let settings = {...this.therblig.parameters};
    this.authrService.setTherbligById({"parameters":settings},this.therbligKey);
  }

  private setDuration(value, key) {
    this.therblig.duration[key] = value;
    let settings = {...this.therblig.duration};
    this.authrService.setTherbligById({"duration":settings},this.therbligKey);
  }

  private setCost(value, key) {
    this.therblig.cost[key] = value;
    let settings = {...this.therblig.cost};
    this.authrService.setTherbligById({"cost":settings},this.therbligKey);
  }

  private setArm(value) {
    this.therblig.parameters.arm = value
    let settings = {...this.therblig.parameters};
    this.authrService.setTherbligById({"parameters":settings},this.therbligKey);
  }

  private setEffort(value) {
    if (value > 1) {
      value = 1;
    } else if (value < 0) {
      value = 0;
    }

    this.therblig.parameters.effort = value;
    let settings = {"effort":this.therblig.parameters.effort};
    this.authrService.setTherbligById(settings,this.therbligKey);
  }

  private get currentAgent() {
    if (this.therblig.parameters.agent == null || this.therblig.parameters.agent == "OPTIMIZE_DIRECTIVE") {
        return this.optAgent;
    } else {
        return this.authrService.getAgentById(this.therblig.parameters.agent);
    }
    return null;
  }

  private get currentThing() {
    if (this.therblig.parameters.thing) {
      return this.authrService.getThingById(this.therblig.parameters.thing);
    } else {
      return this.notsetThing;
    }

    return null;
  }

  private get currentDestination() {
    if (this.therblig.parameters.destination) {
      return this.authrService.getDestinationById(this.therblig.parameters.destination);
    } else {
      return this.notsetDestination;
    }
    return null;
  }

  private get errors(): any {
    this.therblig = this.authrService.getTherbligById(this.therbligKey);
    return this.therblig.errors;
  }

  private get verificationErrors(): string[] {
    let verificationErrors = []
    this.therblig = this.authrService.getTherbligById(this.therbligKey);
    for (let key in this.therblig.errors) {
      if(key != null && (key.substring(key.length-3, key.length) == "set" || key.substring(key.length-7, key.length) == "require")) {
        // console.log(this.therblig.errors[key]);
        this.therblig.errors[key].forEach((error)=>{
          verificationErrors.push(error);
        })
        // verificationErrors.push(this.therblig.errors[key]);
      }
    }
    return verificationErrors;
  }

  private get verificationErrorCount(): number {
    let errors = this.verificationErrors;
    console.log(errors);
    return errors.length;
    // for (let key in this.verificationErrors) {
    //   if (this.verificationErrors[key].length > 0) {
    //     count ++;
    //   }
    // }
    // return count;
  }

  public noElementCheck(type:string) {
      // Check if there are elements of that type available
      if (this.authrService.counts[type] < 1) {
          this.snackbar
          .open("Please define "+type+" in the Setup Tab","Go!", {duration: 2000})
          .onAction().subscribe(() => {this.router.navigate(['/setup',{ outlets: { setup: [type] } }])});
      }
  }

  private deleteTherblig(key) {
      this.router.navigate(['/plan']);
      this.authrService.focused = null;
      this.authrService.deleteTherblig(key);
  }

  trackByFn(index: any, item: any) {
   return index;
  }

}
