import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { TherbligPrimitiveDetailComponent } from './therblig-primitive-detail.component';

describe('TherbligPrimitiveDetailComponent', () => {
  let component: TherbligPrimitiveDetailComponent;
  let fixture: ComponentFixture<TherbligPrimitiveDetailComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ TherbligPrimitiveDetailComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(TherbligPrimitiveDetailComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
