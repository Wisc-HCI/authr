import { Component, OnInit } from '@angular/core';
import { ActivatedRoute, Router } from '@angular/router';
import { AuthrService } from '../services/authr.service';
import { TherbligTypes } from '../models/therblig';
import { TherbligPrimitive } from '../models/therbligPrimitive';

@Component({
  selector: 'app-therblig-primitive-detail',
  templateUrl: './therblig-primitive-detail.component.html',
  styleUrls: ['./therblig-primitive-detail.component.css']
})
export class TherbligPrimitiveDetailComponent implements OnInit {
  public type: string;
  public imgsrc: string;
  public therbligPrimitive: TherbligPrimitive;
  public setsSize: number;

  constructor(public route: ActivatedRoute,
              public router: Router,
              public authrService: AuthrService) { }

  ngOnInit() {
    this.route.paramMap.subscribe(params => {
      this.type = params.get('therblig_primitive_type');
      this.therbligPrimitive = this.authrService.primitives[this.type];
      this.imgsrc = "assets/videos/"+this.type+".gif";
      this.setsSize = 0;
      for(let i in this.therbligPrimitive.sets) {
        this.setsSize++;
      }
    });
  }
}
