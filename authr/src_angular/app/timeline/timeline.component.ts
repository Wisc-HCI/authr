import { Component, OnInit, Input, OnChanges, SimpleChanges } from '@angular/core';
import { Agent } from '../models/agent';
import { AuthrService } from '../services/authr.service';
import { getNotSetAgent } from '../models/notset';
import {TrackComponent} from '../track/track.component';

@Component({
  selector: 'timeline',
  templateUrl: './timeline.component.html',
  styleUrls: ['./timeline.component.css']
})

export class TimelineComponent implements OnInit {

  @Input() data: any;
  @Input() therbligs: string[];
  @Input() scaling: number = 100;
  @Input()
  set time(time: number) {
    this._time = time;
  }
  get time(): number { return this._time; }

  public notsetagent: Agent = getNotSetAgent();

  public tableData: any[];
  public agents: string[] = [];
  public columns: string[] = ['agent','timeline'];
  private _time: number = 0;

  constructor(public authrService:AuthrService) { }

  ngOnInit() {
  }

  handleClick(text) {
    console.log(text)
  }

  ngOnChanges(changes:SimpleChanges) {
    if (this.data) {
      this.tableData = [];
      this.agents = [];
      Object.keys(this.data).forEach((key) => {
        this.agents.push(key);
        this.data[key].forEach((therblig, index) => {
          therblig.width = this.scaling*(therblig.duration);
          if(index == 0) {
            therblig.offset = therblig.start_time;
            therblig.gap = this.scaling*(therblig.offset);
          }
          if(index > 0 && index < this.data[key].length){
            therblig.offset = therblig.start_time - (this.data[key][index-1].start_time + this.data[key][index-1].duration );
            therblig.gap = this.scaling*(therblig.offset);
          }
        });
        this.tableData.push({agent:key,timeline:this.data[key]})
      });
    } else {
      this.tableData = []
    }
  }

}
