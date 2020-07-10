import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { TherbligDetailComponent } from './therblig-detail.component';

describe('TherbligDetailComponent', () => {
  let component: TherbligDetailComponent;
  let fixture: ComponentFixture<TherbligDetailComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ TherbligDetailComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(TherbligDetailComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
