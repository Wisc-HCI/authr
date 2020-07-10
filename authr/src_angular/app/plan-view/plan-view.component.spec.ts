import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { PlanViewComponent } from './plan-view.component';

describe('PlanViewComponent', () => {
  let component: PlanViewComponent;
  let fixture: ComponentFixture<PlanViewComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ PlanViewComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(PlanViewComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
