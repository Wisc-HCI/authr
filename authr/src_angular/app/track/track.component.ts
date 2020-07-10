import { Component, OnInit, Input } from '@angular/core';
import { TimelineComponent } from '../timeline/timeline.component';
import { AuthrService } from "../services/authr.service";

@Component({
  selector: 'app-track',
  templateUrl: './track.component.html',
  styleUrls: ['./track.component.scss']
})
export class TrackComponent implements OnInit {
  @Input() data: any;
  @Input() therbligs: string[];

  private _time: number = 0;

  constructor(public authrService:AuthrService) { }

  ngOnInit() {
  }

  handleClick(event,id) {
    console.log(id);
    this.authrService.focused = id;
    event.stopPropagation();
  }

  isActive(therbligId) {
    return this.therbligs.indexOf(therbligId) >= 0
  }

}
