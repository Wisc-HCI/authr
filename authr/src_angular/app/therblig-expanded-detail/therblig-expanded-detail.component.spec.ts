import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { TherbligExpandedDetailComponent } from './therblig-detail.component';

describe('TherbligExpandedDetailComponent', () => {
  let component: TherbligExpandedDetailComponent;
  let fixture: ComponentFixture<TherbligExpandedDetailComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ TherbligExpandedDetailComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(TherbligExpandedDetailComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
